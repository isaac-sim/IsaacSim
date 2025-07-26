// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
 * @brief Implementation of the ROS 2 factory interface
 * @details
 * This file contains the concrete implementation of the ROS 2 factory interface,
 * providing methods for creating various ROS 2 entities such as nodes, publishers,
 * subscribers, and message types. It serves as the main factory for instantiating
 * ROS 2 components in the Isaac Sim bridge.
 */
#pragma once

#include <isaacsim/ros2/bridge/Ros2Factory.h>

#include <memory>

namespace isaacsim
{
namespace ros2
{
namespace bridge
{

/**
 * @class Ros2FactoryImpl
 * @brief Concrete implementation of the ROS 2 factory interface
 * @details
 * Implements the Ros2Factory interface to provide creation methods for all
 * ROS 2 entities used in the Isaac Sim bridge. This includes context handlers,
 * node handlers, communication primitives, and various message types.
 */
class Ros2FactoryImpl : public Ros2Factory
{
public:
    /**
     * @brief Creates a new ROS 2 context handle
     * @return std::shared_ptr<Ros2ContextHandle> Pointer to the created context handle
     */
    virtual std::shared_ptr<Ros2ContextHandle> createContextHandle();

    /**
     * @brief Creates a new ROS 2 node handle
     * @param[in] name Name of the node
     * @param[in] namespaceName Namespace for the node
     * @param[in] contextHandle Pointer to the context handle for this node
     * @return std::shared_ptr<Ros2NodeHandle> Pointer to the created node handle
     */
    virtual std::shared_ptr<Ros2NodeHandle> createNodeHandle(const char* name,
                                                             const char* namespaceName,
                                                             Ros2ContextHandle* contextHandle);

    /**
     * @brief Creates a new ROS 2 publisher
     * @param[in] nodeHandle Pointer to the node handle that will own this publisher
     * @param[in] topicName Name of the topic to publish to
     * @param[in] typeSupport Type support handle for the message type
     * @param[in] qos Quality of Service settings for the publisher
     * @return std::shared_ptr<Ros2Publisher> Pointer to the created publisher
     */
    virtual std::shared_ptr<Ros2Publisher> createPublisher(Ros2NodeHandle* nodeHandle,
                                                           const char* topicName,
                                                           const void* typeSupport,
                                                           const Ros2QoSProfile& qos);

    /**
     * @brief Creates a new ROS 2 subscriber
     * @param[in] nodeHandle Pointer to the node handle that will own this subscriber
     * @param[in] topicName Name of the topic to subscribe to
     * @param[in] typeSupport Type support handle for the message type
     * @param[in] qos Quality of Service settings for the subscriber
     * @return std::shared_ptr<Ros2Subscriber> Pointer to the created subscriber
     */
    virtual std::shared_ptr<Ros2Subscriber> createSubscriber(Ros2NodeHandle* nodeHandle,
                                                             const char* topicName,
                                                             const void* typeSupport,
                                                             const Ros2QoSProfile& qos);

    /**
     * @brief Creates a new ROS 2 service server
     * @param[in] nodeHandle Pointer to the node handle that will own this service
     * @param[in] serviceName Name of the service
     * @param[in] typeSupport Type support handle for the service type
     * @param[in] qos Quality of Service settings for the service
     * @return std::shared_ptr<Ros2Service> Pointer to the created service server
     */
    virtual std::shared_ptr<Ros2Service> createService(Ros2NodeHandle* nodeHandle,
                                                       const char* serviceName,
                                                       const void* typeSupport,
                                                       const Ros2QoSProfile& qos);

    /**
     * @brief Creates a new ROS 2 service client
     * @param[in] nodeHandle Pointer to the node handle that will own this client
     * @param[in] serviceName Name of the service to connect to
     * @param[in] typeSupport Type support handle for the service type
     * @param[in] qos Quality of Service settings for the client
     * @return std::shared_ptr<Ros2Client> Pointer to the created service client
     */
    virtual std::shared_ptr<Ros2Client> createClient(Ros2NodeHandle* nodeHandle,
                                                     const char* serviceName,
                                                     const void* typeSupport,
                                                     const Ros2QoSProfile& qos);

    // Message creation methods
    virtual std::shared_ptr<Ros2ClockMessage> createClockMessage(); /**< Creates a Clock message */
    virtual std::shared_ptr<Ros2ImuMessage> createImuMessage(); /**< Creates an IMU message */
    virtual std::shared_ptr<Ros2CameraInfoMessage> createCameraInfoMessage(); /**< Creates a Camera Info message */
    virtual std::shared_ptr<Ros2ImageMessage> createImageMessage(); /**< Creates an Image message */
    virtual std::shared_ptr<Ros2NitrosBridgeImageMessage> createNitrosBridgeImageMessage(); /**< Creates a Nitros Bridge
                                                                                               Image message */
    virtual std::shared_ptr<Ros2BoundingBox2DMessage> createBoundingBox2DMessage(); /**< Creates a 2D Bounding Box
                                                                                       message */
    virtual std::shared_ptr<Ros2BoundingBox3DMessage> createBoundingBox3DMessage(); /**< Creates a 3D Bounding Box
                                                                                       message */
    virtual std::shared_ptr<Ros2OdometryMessage> createOdometryMessage(); /**< Creates an Odometry message */
    virtual std::shared_ptr<Ros2RawTfTreeMessage> createRawTfTreeMessage(); /**< Creates a Raw Transform Tree message */
    virtual std::shared_ptr<Ros2SemanticLabelMessage> createSemanticLabelMessage(); /**< Creates a Semantic Label
                                                                                       message */
    virtual std::shared_ptr<Ros2JointStateMessage> createJointStateMessage(); /**< Creates a Joint State message */
    virtual std::shared_ptr<Ros2PointCloudMessage> createPointCloudMessage(); /**< Creates a Point Cloud message */
    virtual std::shared_ptr<Ros2LaserScanMessage> createLaserScanMessage(); /**< Creates a Laser Scan message */
    virtual std::shared_ptr<Ros2TfTreeMessage> createTfTreeMessage(); /**< Creates a Transform Tree message */
    virtual std::shared_ptr<Ros2TwistMessage> createTwistMessage(); /**< Creates a Twist message */
    virtual std::shared_ptr<Ros2AckermannDriveStampedMessage> createAckermannDriveStampedMessage(); /**< Creates an
                                                                                                       Ackermann Drive
                                                                                                       Stamped message
                                                                                                     */

    /**
     * @brief Creates a dynamic message type at runtime
     * @param[in] pkgName Name of the ROS 2 package containing the message definition
     * @param[in] msgSubfolder Subfolder containing the message definition (msg, srv, action)
     * @param[in] msgName Name of the message type
     * @param[in] messageType Type of the message (topic, service, action component)
     * @return std::shared_ptr<Ros2Message> Pointer to the created dynamic message
     */
    virtual std::shared_ptr<Ros2Message> createDynamicMessage(const std::string& pkgName,
                                                              const std::string& msgSubfolder,
                                                              const std::string& msgName,
                                                              BackendMessageType messageType = BackendMessageType::eMessage);

    /**
     * @brief Validates a ROS 2 topic name
     * @param[in] topicName Name to validate
     * @return bool True if the topic name is valid
     */
    virtual bool validateTopicName(const std::string& topicName);

    /**
     * @brief Validates a ROS 2 namespace name
     * @param[in] namespaceName Name to validate
     * @return bool True if the namespace name is valid
     */
    virtual bool validateNamespaceName(const std::string& namespaceName);

    /**
     * @brief Validates a ROS 2 node name
     * @param[in] nodeName Name to validate
     * @return bool True if the node name is valid
     */
    virtual bool validateNodeName(const std::string& nodeName);
};

} // namespace bridge
} // namespace ros2
} // namespace isaacsim

/**
 * @brief Factory creation function exported by the library
 * @details
 * Platform-specific export of the factory creation function.
 * This function is called to create a new instance of the ROS 2 factory.
 *
 * @return std::unique_ptr<isaacsim::ros2::bridge::Ros2Factory> Unique pointer to the created factory
 */
#ifdef _MSC_VER
__declspec(dllexport) std::unique_ptr<isaacsim::ros2::bridge::Ros2Factory> createFactory();
#else
std::unique_ptr<isaacsim::ros2::bridge::Ros2Factory> createFactory();
#endif

/**
 * @brief C wrapper for factory creation function for dynamic symbol loading
 * @details
 * This function provides a C interface for dynamic symbol loading.
 * It returns a raw pointer that should be managed by the caller.
 *
 * @return isaacsim::ros2::bridge::Ros2Factory* Raw pointer to the created factory
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) isaacsim::ros2::bridge::Ros2Factory* createFactoryC();
#else
extern "C" isaacsim::ros2::bridge::Ros2Factory* createFactoryC();
#endif
