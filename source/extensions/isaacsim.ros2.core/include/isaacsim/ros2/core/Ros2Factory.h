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

/** @file
 * @brief Factory that contains the class definitions and methods for creating ROS 2 entities
 * @details
 * This header file defines the factory class and related types for creating and managing ROS 2
 * entities according to the sourced ROS 2 distribution. It provides comprehensive support for
 * dynamic message handling, transformations, and various ROS 2 message types.
 */
#pragma once

#include <isaacsim/core/includes/LibraryLoader.h>
#include <isaacsim/core/includes/Math.h>
#include <isaacsim/ros2/core/Ros2Message.h>
#include <isaacsim/ros2/core/Ros2QoS.h>
#include <isaacsim/ros2/core/Ros2Types.h>

#include <memory>
#include <string>
#include <vector>


namespace isaacsim
{
namespace ros2
{
namespace core
{


/**
 * @class Ros2Factory
 * @brief Base class for creating ROS 2 related functions/objects according to the sourced ROS 2 distribution.
 * @details
 * This abstract factory class provides an interface for creating various ROS 2 entities such as nodes,
 * publishers, subscribers, and different message types. It serves as a bridge between Isaac Sim and
 * ROS 2, allowing simulation components to communicate with ROS 2 ecosystem.
 *
 * Implementations of this class handle the specific ROS 2 distribution being used (e.g., Foxy, Humble)
 * while providing a consistent interface for the simulation.
 */
class Ros2Factory
{
public:
    /**
     * @brief Virtual destructor.
     * @details
     * Ensures proper cleanup of derived classes when destroyed through a base class pointer.
     */
    virtual ~Ros2Factory() = default;

    /**
     * @brief Create a ROS 2 context handler.
     * @details
     * Creates and initializes a ROS 2 context which is required for any ROS 2 communication.
     * The context maintains the ROS 2 initialization state and global configuration.
     *
     * @return Shared pointer to the created context handler.
     */
    virtual std::shared_ptr<Ros2ContextHandle> createContextHandle() = 0;

    /**
     * @brief Create a ROS 2 node handler.
     * @details
     * Creates a ROS 2 node with the specified name and namespace within the given context.
     * The node is the primary entry point for communication with the ROS 2 graph.
     *
     * @param[in] name Name of the node.
     * @param[in] namespaceName Namespace of the node.
     * @param[in] contextHandle Context handler within which to create the node.
     * @return Shared pointer to the created node handler.
     */
    virtual std::shared_ptr<Ros2NodeHandle> createNodeHandle(const char* name,
                                                             const char* namespaceName,
                                                             Ros2ContextHandle* contextHandle) = 0;

    /**
     * @brief Create a ROS 2 publisher.
     * @details
     * Creates a publisher that can send messages of the specified type on the given topic
     * with the specified quality of service settings.
     *
     * @param[in] nodeHandle Node handler to which the publisher will be attached.
     * @param[in] topicName Name of the topic to publish on.
     * @param[in] typeSupport Message type support structure that provides serialization capabilities.
     * @param[in] qos Quality of service profile that defines communication properties.
     * @return Shared pointer to the created publisher.
     */
    virtual std::shared_ptr<Ros2Publisher> createPublisher(Ros2NodeHandle* nodeHandle,
                                                           const char* topicName,
                                                           const void* typeSupport,
                                                           const Ros2QoSProfile& qos) = 0;

    /**
     * @brief Create a ROS 2 subscriber.
     * @details
     * Creates a subscriber that can receive messages of the specified type from the given topic
     * with the specified quality of service settings.
     *
     * @param[in] nodeHandle Node handler to which the subscriber will be attached.
     * @param[in] topicName Name of the topic to subscribe to.
     * @param[in] typeSupport Message type support structure that provides deserialization capabilities.
     * @param[in] qos Quality of Service profile that defines communication properties.
     * @return Shared pointer to the created subscriber.
     */
    virtual std::shared_ptr<Ros2Subscriber> createSubscriber(Ros2NodeHandle* nodeHandle,
                                                             const char* topicName,
                                                             const void* typeSupport,
                                                             const Ros2QoSProfile& qos) = 0;

    /**
     * @brief Create a ROS 2 service server.
     * @details
     * Creates a service server that can receive service requests and provide responses
     * of the specified type on the given service name with the specified quality of service settings.
     *
     * @param[in] nodeHandle Node handler to which the service server will be attached.
     * @param[in] serviceName Name of the service.
     * @param[in] typeSupport Service type support structure that provides serialization/deserialization capabilities.
     * @param[in] qos Quality of Service profile that defines communication properties.
     * @return Shared pointer to the created service server.
     */
    virtual std::shared_ptr<Ros2Service> createService(Ros2NodeHandle* nodeHandle,
                                                       const char* serviceName,
                                                       const void* typeSupport,
                                                       const Ros2QoSProfile& qos) = 0;

    /**
     * @brief Create a ROS 2 service client.
     * @details
     * Creates a service client that can send service requests and receive responses
     * of the specified type to the given service name with the specified quality of service settings.
     *
     * @param[in] nodeHandle Node handler to which the service client will be attached.
     * @param[in] serviceName Name of the service.
     * @param[in] typeSupport Service type support structure that provides serialization/deserialization capabilities.
     * @param[in] qos Quality of Service profile that defines communication properties.
     * @return Shared pointer to the created service client.
     */
    virtual std::shared_ptr<Ros2Client> createClient(Ros2NodeHandle* nodeHandle,
                                                     const char* serviceName,
                                                     const void* typeSupport,
                                                     const Ros2QoSProfile& qos) = 0;

    /**
     * @brief Create a ROS 2 `rosgraph_msgs/msg/Clock` message.
     * @details
     * Creates a message that can be used to publish simulation time to ROS 2 clock topics.
     *
     * @return Shared pointer to the created clock message.
     */
    virtual std::shared_ptr<Ros2ClockMessage> createClockMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/Imu` message.
     * @details
     * Creates a message that can be used to publish inertial measurement unit data.
     *
     * @return Shared pointer to the created IMU message.
     */
    virtual std::shared_ptr<Ros2ImuMessage> createImuMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/CameraInfo` message.
     * @details
     * Creates a message that can be used to publish camera calibration and metadata.
     *
     * @return Shared pointer to the created camera info message.
     */
    virtual std::shared_ptr<Ros2CameraInfoMessage> createCameraInfoMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/Image` message.
     * @details
     * Creates a message that can be used to publish image data.
     *
     * @return Shared pointer to the created image message.
     */
    virtual std::shared_ptr<Ros2ImageMessage> createImageMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/CompressedImage` message.
     * @details
     * Creates a message that can be used to publish compressed image data (e.g., H264, HEVC).
     *
     * @return Shared pointer to the created compressed image message.
     */
    virtual std::shared_ptr<Ros2CompressedImageMessage> createCompressedImageMessage() = 0;

    /**
     * @brief Create a ROS 2 `isaac_ros_nitros_bridge_interfaces/msg/NitrosBridgeImage` message.
     * @details
     * Creates a message that can be used for transferring images between NITROS and ROS 2.
     *
     * @return Shared pointer to the created NITROS bridge image message.
     */
    virtual std::shared_ptr<Ros2NitrosBridgeImageMessage> createNitrosBridgeImageMessage() = 0;

    /**
     * @brief Create a ROS 2 `vision_msgs/msg/Detection2DArray` message.
     * @details
     * Creates a message that can be used to publish 2D bounding box detection results.
     *
     * @return Shared pointer to the created 2D bounding box message.
     */
    virtual std::shared_ptr<Ros2BoundingBox2DMessage> createBoundingBox2DMessage() = 0;

    /**
     * @brief Create a ROS 2 `vision_msgs/msg/Detection3DArray` message.
     * @details
     * Creates a message that can be used to publish 3D bounding box detection results.
     *
     * @return Shared pointer to the created 3D bounding box message.
     */
    virtual std::shared_ptr<Ros2BoundingBox3DMessage> createBoundingBox3DMessage() = 0;

    /**
     * @brief Create a ROS 2 `nav_msgs/msg/Odometry` message.
     * @details
     * Creates a message that can be used to publish position and velocity in a global coordinate frame.
     *
     * @return Shared pointer to the created odometry message.
     */
    virtual std::shared_ptr<Ros2OdometryMessage> createOdometryMessage() = 0;

    /**
     * @brief Create a ROS 2 `tf2_msgs/msg/TFMessage` message.
     * @details
     * Creates a raw transform message that can be used to publish coordinate frame transformations.
     *
     * @return Shared pointer to the created raw transform tree message.
     */
    virtual std::shared_ptr<Ros2RawTfTreeMessage> createRawTfTreeMessage() = 0;

    /**
     * @brief Create a ROS 2 `std_msgs/msg/String` message as semantic label.
     * @details
     * Creates a message that can be used to publish semantic labels for objects.
     *
     * @return Shared pointer to the created semantic label message.
     */
    virtual std::shared_ptr<Ros2SemanticLabelMessage> createSemanticLabelMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/JointState` message.
     * @details
     * Creates a message that can be used to publish joint states including position, velocity, and effort.
     *
     * @return Shared pointer to the created joint state message.
     */
    virtual std::shared_ptr<Ros2JointStateMessage> createJointStateMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/PointCloud2` message.
     * @details
     * Creates a message that can be used to publish point cloud data.
     *
     * @return Shared pointer to the created point cloud message.
     */
    virtual std::shared_ptr<Ros2PointCloudMessage> createPointCloudMessage() = 0;

    /**
     * @brief Create a ROS 2 `sensor_msgs/msg/LaserScan` message.
     * @details
     * Creates a message that can be used to publish laser scan data.
     *
     * @return Shared pointer to the created laser scan message.
     */
    virtual std::shared_ptr<Ros2LaserScanMessage> createLaserScanMessage() = 0;

    /**
     * @brief Create a ROS 2 `tf2_msgs/msg/TFMessage` message.
     * @details
     * Creates a transform message that can be used to publish coordinate frame transformations.
     *
     * @return Shared pointer to the created transform tree message.
     */
    virtual std::shared_ptr<Ros2TfTreeMessage> createTfTreeMessage() = 0;

    /**
     * @brief Create a ROS 2 `geometry_msgs/msg/Twist` message.
     * @details
     * Creates a message that can be used to represent velocity in free space with linear and angular components.
     *
     * @return Shared pointer to the created twist message.
     */
    virtual std::shared_ptr<Ros2TwistMessage> createTwistMessage() = 0;

    /**
     * @brief Create a ROS 2 `ackermann_msgs/msg/AckermannDriveStamped` message.
     * @details
     * Creates a message that can be used to represent an Ackermann steering command with timestamp.
     *
     * @return Shared pointer to the created Ackermann drive stamped message.
     */
    virtual std::shared_ptr<Ros2AckermannDriveStampedMessage> createAckermannDriveStampedMessage() = 0;

    /**
     * @brief Create a ROS 2 dynamic message.
     * @details
     * Creates a message based on dynamically loaded type information, allowing for handling
     * of message types that may not be known at compile time. This is useful for creating
     * message instances for custom or third-party message types.
     *
     * @param[in] pkgName Message package name (e.g.: `"std_msgs"` for `std_msgs/msg/Int32`).
     * @param[in] msgSubfolder Message subfolder name (e.g.: `"msg"` for `std_msgs/msg/Int32`).
     * @param[in] msgName Message name (e.g.: `"Int32"` for `std_msgs/msg/Int32`).
     * @param[in] messageType Message type, defaults to eMessage.
     * @return Shared pointer to the created dynamic message.
     */
    virtual std::shared_ptr<Ros2Message> createDynamicMessage(
        const std::string& pkgName,
        const std::string& msgSubfolder,
        const std::string& msgName,
        BackendMessageType messageType = BackendMessageType::eMessage) = 0;

    /**
     * @brief Determine if the given topic name is valid.
     * @details
     * Validates a topic name according to ROS 2 naming conventions.
     *
     * @param[in] topicName Topic name to validate.
     * @return True if the topic name is valid, false otherwise.
     */
    virtual bool validateTopicName(const std::string& topicName) = 0;

    /**
     * @brief Determine if the given node namespace name is valid.
     * @details
     * Validates a node namespace name according to ROS 2 naming conventions.
     *
     * @param[in] namespaceName Namespace name to validate.
     * @return True if the node namespace name is valid, false otherwise.
     */
    virtual bool validateNamespaceName(const std::string& namespaceName) = 0;

    /**
     * @brief Determine if the given node name is valid.
     * @details
     * Validates a node name according to ROS 2 naming conventions.
     *
     * @param[in] nodeName Node name to validate.
     * @return True if the node name is valid, false otherwise.
     */
    virtual bool validateNodeName(const std::string& nodeName) = 0;
};

} // namespace core
} // namespace ros2
} // namespace isaacsim
