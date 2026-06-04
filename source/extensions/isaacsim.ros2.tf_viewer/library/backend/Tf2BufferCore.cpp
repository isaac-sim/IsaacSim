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

#include "Tf2Impl.h"

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @brief Default constructor for Ros2BufferCoreImpl.
 */
Ros2BufferCoreImpl::Ros2BufferCoreImpl() = default;

/**
 * @brief Default destructor for Ros2BufferCoreImpl.
 */
Ros2BufferCoreImpl::~Ros2BufferCoreImpl() = default;

/**
 * @brief Sets a transform in the buffer from a ROS TF message.
 * @details
 * Processes a TFMessage by extracting each transform, converting it to the
 * appropriate C++ type, and adding it to the internal buffer. If any transform
 * cannot be set, the function returns false.
 *
 * @param[in] msg Pointer to a tf2_msgs__msg__TFMessage message.
 * @param[in] authority String identifying the source of the transform.
 * @param[in] isStatic Boolean indicating if the transform is static (unchanging over time).
 * @return True if all transforms were successfully set, false otherwise.
 * @throws tf2::TransformException May throw exceptions from the underlying buffer.
 */
bool Ros2BufferCoreImpl::setTransform(void* msg, const std::string& authority, bool isStatic)
{
    if (!msg)
    {
        return false;
    }
    tf2_msgs__msg__TFMessage* tfMsg = static_cast<tf2_msgs__msg__TFMessage*>(msg);
    for (size_t i = 0; i < tfMsg->transforms.size; ++i)
    {
        // geometry_msgs/TransformStamped's C to C++ conversion
        auto data = tfMsg->transforms.data[i];
        auto transformStamped = geometry_msgs::msg::TransformStamped();
        transformStamped.header.set__frame_id(std::string(data.header.frame_id.data));
        transformStamped.header.stamp.set__sec(data.header.stamp.sec);
        transformStamped.header.stamp.set__nanosec(data.header.stamp.nanosec);
        transformStamped.set__child_frame_id(std::string(data.child_frame_id.data));
        transformStamped.transform.translation.set__x(data.transform.translation.x);
        transformStamped.transform.translation.set__y(data.transform.translation.y);
        transformStamped.transform.translation.set__z(data.transform.translation.z);
        transformStamped.transform.rotation.set__w(data.transform.rotation.w);
        transformStamped.transform.rotation.set__x(data.transform.rotation.x);
        transformStamped.transform.rotation.set__y(data.transform.rotation.y);
        transformStamped.transform.rotation.set__z(data.transform.rotation.z);
        // call tf2::BufferCore setTransform method
        try
        {
            m_buffer.setTransform(transformStamped, authority, isStatic);
        }
        catch (tf2::TransformException& ex)
        {
            std::string errorString = ex.what();
            CARB_LOG_ERROR("%s", errorString.c_str());
            return false;
        }
    }
    return true;
}

/**
 * @brief Gets the transform between two frames at time zero.
 * @details
 * Looks up the transform from source_frame to target_frame at time zero and
 * populates the translation and rotation arrays with the result.
 *
 * @param[in] targetFrame Frame ID to which data should be transformed.
 * @param[in] sourceFrame Frame ID where the data originated.
 * @param[out] translation Array to store the translation components [x, y, z].
 * @param[out] rotation Array to store the rotation components as quaternion [x, y, z, w].
 * @return True if transform was successfully retrieved, false otherwise.
 * @throws Various tf2 exceptions that are caught internally.
 */
bool Ros2BufferCoreImpl::getTransform(const std::string& targetFrame,
                                      const std::string& sourceFrame,
                                      double translation[],
                                      double rotation[])
{
    try
    {
        auto transformStamped = m_buffer.lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
        translation[0] = transformStamped.transform.translation.x;
        translation[1] = transformStamped.transform.translation.y;
        translation[2] = transformStamped.transform.translation.z;
        rotation[0] = transformStamped.transform.rotation.x;
        rotation[1] = transformStamped.transform.rotation.y;
        rotation[2] = transformStamped.transform.rotation.z;
        rotation[3] = transformStamped.transform.rotation.w;
    }
    catch (const tf2::LookupException& e)
    {
        (void)e; // avoid warning C4101: unreferenced local variable
        return false;
    }
    catch (const tf2::ConnectivityException& e)
    {
        (void)e; // avoid warning C4101: unreferenced local variable
        return false;
    }
    catch (const tf2::ExtrapolationException& e)
    {
        (void)e; // avoid warning C4101: unreferenced local variable
        return false;
    }
    catch (const tf2::InvalidArgumentException& e)
    {
        (void)e; // avoid warning C4101: unreferenced local variable
        return false;
    }
    catch (...)
    {
        std::string errorString = "UNKNOW EXCEPTION";
        CARB_LOG_ERROR("%s", errorString.c_str());
        return false;
    }
    return true;
}

/**
 * @brief Gets the parent frame of a specified frame at time zero.
 * @details
 * Queries the buffer to find the parent frame of the specified frame.
 *
 * @param[in] frame Frame ID to find the parent of.
 * @param[out] parentFrame Reference that will be populated with the parent frame ID.
 * @return True if the parent frame was found, false otherwise.
 */
bool Ros2BufferCoreImpl::getParentFrame(const std::string& frame, std::string& parentFrame)
{
    return m_buffer._getParent(frame, tf2::TimePointZero, parentFrame);
}

/**
 * @brief Gets all available frame IDs in the buffer.
 * @details
 * Retrieves a list of all frame IDs that are currently in the transform buffer.
 *
 * @return Vector containing all frame IDs.
 */
std::vector<std::string> Ros2BufferCoreImpl::getFrames()
{
    m_frames.clear();
    m_buffer._getFrameStrings(m_frames);
    return m_frames;
}

/**
 * @brief Clears all transforms from the buffer.
 * @details
 * Removes all stored transformations from the internal buffer.
 */
void Ros2BufferCoreImpl::clear()
{
    m_buffer.clear();
}

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim
