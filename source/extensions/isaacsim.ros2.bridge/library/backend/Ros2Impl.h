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
 * @brief Implementation classes for ROS 2 message handling and communication
 * @details
 * This file contains the concrete implementations of various ROS 2 message types,
 * communication primitives, and utility classes used in the Isaac Sim ROS 2 bridge.
 * It provides the backend functionality for message serialization, deserialization,
 * and ROS 2 communication handling.
 */
#pragma once

#if !defined(_WIN32)
#    include "isaac_ros_nitros_bridge_interfaces/msg/nitros_bridge_image.h"
#endif

#include "ackermann_msgs/msg/ackermann_drive_stamped.h"
#include "builtin_interfaces/msg/time.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "sensor_msgs/msg/image.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/joint_state.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "sensor_msgs/msg/point_field.h"
#include "tf2_msgs/msg/tf_message.h"
#include "vision_msgs/msg/detection2_d.h"
#include "vision_msgs/msg/detection2_d_array.h"
#include "vision_msgs/msg/detection3_d.h"
#include "vision_msgs/msg/detection3_d_array.h"
#include "vision_msgs/msg/object_hypothesis_with_pose.h"

#include <isaacsim/ros2/bridge/Ros2FactoryImpl.h>
#include <nlohmann/json.hpp>
#include <omni/physics/tensors/IArticulationView.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rosgraph_msgs/msg/clock.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
namespace isaacsim
{
namespace ros2
{
namespace bridge
{

/**
 * @class Ros2MessageInterfaceImpl
 * @brief Implementation of the ROS 2 message interface
 * @details
 * Provides concrete implementation for ROS 2 message handling, including
 * time, string, and header field management. This class serves as a base
 * for specific message type implementations.
 */
class Ros2MessageInterfaceImpl : public Ros2MessageInterface
{
public:
    /**
     * @brief Constructor for message interface implementation
     * @param[in] pkgName ROS 2 package name containing the message definition
     * @param[in] msgSubfolder Message subfolder (msg, srv, action)
     * @param[in] msgName Name of the message type
     * @param[in] messageType Type of message (default: standard message)
     * @param[in] showLoadingError Whether to show library loading errors
     */
    Ros2MessageInterfaceImpl(std::string pkgName,
                             std::string msgSubfolder,
                             std::string msgName,
                             BackendMessageType messageType = BackendMessageType::eMessage,
                             bool showLoadingError = false);
    /**
     * @brief Writes a ROS 2 time field
     * @param[in] nanoseconds Time in nanoseconds
     * @param[out] time ROS 2 time structure to fill
     */
    void writeRosTime(const int64_t nanoseconds, builtin_interfaces__msg__Time& time);
    /**
     * @brief Writes a ROS 2 string field
     * @param[in] input String to write
     * @param[out] output ROS 2 string structure to fill
     */
    void writeRosString(const std::string& input, rosidl_runtime_c__String& output);
    /**
     * @brief Writes a ROS 2 header field
     * @param[in] frameId Frame ID for the header
     * @param[in] nanoseconds Timestamp in nanoseconds
     * @param[out] header ROS 2 header structure to fill
     */
    void writeRosHeader(const std::string& frameId, const int64_t nanoseconds, std_msgs__msg__Header& header);
};

/**
 * @class Ros2ClockMessageImpl
 * @brief Implementation of ROS 2 Clock message
 * @details
 * Handles the creation and manipulation of rosgraph_msgs/msg/Clock messages,
 * which are used for time synchronization in ROS 2.
 */
class Ros2ClockMessageImpl : public Ros2ClockMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2ClockMessageImpl();
    virtual ~Ros2ClockMessageImpl();
    virtual const void* getTypeSupportHandle();
    virtual void readData(double& timeStamp);
    virtual void writeData(double timeStamp);
};

/**
 * @class Ros2ImuMessageImpl
 * @brief Implementation of ROS 2 IMU message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/Imu messages,
 * which contain inertial measurement unit data including orientation,
 * angular velocity, and linear acceleration.
 */
class Ros2ImuMessageImpl : public Ros2ImuMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2ImuMessageImpl();
    virtual ~Ros2ImuMessageImpl();
    virtual const void* getTypeSupportHandle();
    virtual void writeHeader(double timeStamp, std::string& frameId);
    virtual void writeAcceleration(bool covariance, const std::vector<double>& acceleration);
    virtual void writeVelocity(bool covariance, const std::vector<double>& velocity);
    virtual void writeOrientation(bool covariance, const std::vector<double>& orientation);
};

/**
 * @class Ros2CameraInfoMessageImpl
 * @brief Implementation of ROS 2 Camera Info message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/CameraInfo messages,
 * which contain camera calibration data and image metadata.
 */
class Ros2CameraInfoMessageImpl : public Ros2CameraInfoMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2CameraInfoMessageImpl();
    virtual ~Ros2CameraInfoMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the camera
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Sets the image resolution
     * @param[in] height Image height in pixels
     * @param[in] width Image width in pixels
     */
    virtual void writeResolution(const uint32_t height, const uint32_t width);

    /**
     * @brief Sets the camera intrinsic matrix
     * @param[in] array Flattened 3x3 intrinsic matrix
     * @param[in] arraySize Size of the array (should be 9)
     */
    virtual void writeIntrinsicMatrix(const double array[], const size_t arraySize);

    /**
     * @brief Sets the camera projection matrix
     * @param[in] array Flattened 3x4 projection matrix
     * @param[in] arraySize Size of the array (should be 12)
     */
    virtual void writeProjectionMatrix(const double array[], const size_t arraySize);

    /**
     * @brief Sets the rectification matrix
     * @param[in] array Flattened 3x3 rectification matrix
     * @param[in] arraySize Size of the array (should be 9)
     */
    virtual void writeRectificationMatrix(const double array[], const size_t arraySize);

    /**
     * @brief Sets the distortion parameters
     * @param[in] array Vector of distortion coefficients
     * @param[in] distortionModel Name of the distortion model
     */
    virtual void writeDistortionParameters(std::vector<double>& array, const std::string& distortionModel);
};

/**
 * @class Ros2ImageMessageImpl
 * @brief Implementation of ROS 2 Image message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/Image messages,
 * which contain raw or compressed image data.
 */
class Ros2ImageMessageImpl : public Ros2ImageMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2ImageMessageImpl();
    virtual ~Ros2ImageMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the image
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Allocates and initializes the image buffer
     * @param[in] height Image height in pixels
     * @param[in] width Image width in pixels
     * @param[in] encoding Image encoding format (e.g., "rgb8", "bgr8")
     */
    virtual void generateBuffer(const uint32_t height, const uint32_t width, const std::string& encoding);
};

/**
 * @class Ros2NitrosBridgeImageMessageImpl
 * @brief Implementation of NITROS Bridge Image message
 * @details
 * Handles the creation and manipulation of isaac_ros_nitros_bridge_interfaces/msg/NitrosBridgeImage
 * messages, which are used for efficient image data transfer in the NVIDIA Isaac ROS pipeline.
 */
class Ros2NitrosBridgeImageMessageImpl : public Ros2NitrosBridgeImageMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2NitrosBridgeImageMessageImpl();
    virtual ~Ros2NitrosBridgeImageMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the image
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Initializes the image metadata
     * @param[in] height Image height in pixels
     * @param[in] width Image width in pixels
     * @param[in] encoding Image encoding format
     */
    virtual void generateBuffer(const uint32_t height, const uint32_t width, const std::string& encoding);

    /**
     * @brief Sets the CUDA memory block data
     * @param[in] imageData Vector containing process ID and CUDA memory block file descriptor
     */
    virtual void writeData(const std::vector<int32_t>& imageData);
};

/**
 * @class Ros2BoundingBox2DMessageImpl
 * @brief Implementation of ROS 2 2D Bounding Box message
 * @details
 * Handles the creation and manipulation of vision_msgs/msg/Detection2DArray messages,
 * which contain 2D object detection results.
 */
class Ros2BoundingBox2DMessageImpl : public Ros2BoundingBox2DMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2BoundingBox2DMessageImpl();
    virtual ~Ros2BoundingBox2DMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the detections
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Sets the bounding box data
     * @param[in] bboxArray Array of 2D bounding box data
     * @param[in] numBoxes Number of bounding boxes in the array
     */
    virtual void writeBboxData(const void* bboxArray, size_t numBoxes);
};

/**
 * @class Ros2BoundingBox3DMessageImpl
 * @brief Implementation of ROS 2 3D Bounding Box message
 * @details
 * Handles the creation and manipulation of vision_msgs/msg/Detection3DArray messages,
 * which contain 3D object detection results.
 */
class Ros2BoundingBox3DMessageImpl : public Ros2BoundingBox3DMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2BoundingBox3DMessageImpl();
    virtual ~Ros2BoundingBox3DMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the detections
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Sets the bounding box data
     * @param[in] bboxArray Array of 3D bounding box data
     * @param[in] numBoxes Number of bounding boxes in the array
     */
    virtual void writeBboxData(const void* bboxArray, size_t numBoxes);
};

/**
 * @class Ros2OdometryMessageImpl
 * @brief Implementation of ROS 2 Odometry message
 * @details
 * Handles the creation and manipulation of nav_msgs/msg/Odometry messages,
 * which contain robot pose and velocity information.
 */
class Ros2OdometryMessageImpl : public Ros2OdometryMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2OdometryMessageImpl();
    virtual ~Ros2OdometryMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the odometry data
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Sets the odometry data
     * @param[in] childFrame Child frame ID for the transform
     * @param[in] linearVelocity Linear velocity vector
     * @param[in] angularVelocity Angular velocity vector
     * @param[in] robotFront Robot's front direction vector
     * @param[in] robotSide Robot's side direction vector
     * @param[in] robotUp Robot's up direction vector
     * @param[in] unitScale Scale factor for unit conversion
     * @param[in] position Robot's position vector
     * @param[in] orientation Robot's orientation quaternion
     * @param[in] publishRawVelocities Whether to publish raw velocities
     */
    virtual void writeData(std::string& childFrame,
                           const pxr::GfVec3d& linearVelocity,
                           const pxr::GfVec3d& angularVelocity,
                           const pxr::GfVec3d& robotFront,
                           const pxr::GfVec3d& robotSide,
                           const pxr::GfVec3d& robotUp,
                           double unitScale,
                           const pxr::GfVec3d& position,
                           const pxr::GfQuatd& orientation,
                           bool publishRawVelocities);
};

/**
 * @class Ros2RawTfTreeMessageImpl
 * @brief Implementation of ROS 2 Raw Transform Tree message
 * @details
 * Handles the creation and manipulation of tf2_msgs/msg/TFMessage messages
 * containing a single transform. Used for publishing raw transform data
 * without additional processing.
 */
class Ros2RawTfTreeMessageImpl : public Ros2RawTfTreeMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2RawTfTreeMessageImpl();
    virtual ~Ros2RawTfTreeMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Sets the transform data
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Parent frame ID
     * @param[in] childFrame Child frame ID
     * @param[in] translation Translation vector
     * @param[in] rotation Rotation quaternion
     */
    virtual void writeData(const double timeStamp,
                           const std::string& frameId,
                           const std::string& childFrame,
                           const pxr::GfVec3d& translation,
                           const pxr::GfQuatd& rotation);
};

/**
 * @class Ros2SemanticLabelMessageImpl
 * @brief Implementation of ROS 2 Semantic Label message
 * @details
 * Handles the creation and manipulation of std_msgs/msg/String messages
 * used for semantic labeling in perception tasks.
 */
class Ros2SemanticLabelMessageImpl : public Ros2SemanticLabelMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2SemanticLabelMessageImpl();
    virtual ~Ros2SemanticLabelMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Sets the semantic label data
     * @param[in] data String containing the semantic label
     */
    virtual void writeData(const std::string& data);
};

/**
 * @class Ros2JointStateMessageImpl
 * @brief Implementation of ROS 2 Joint State message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/JointState messages,
 * which contain robot joint positions, velocities, and efforts.
 */
class Ros2JointStateMessageImpl : public Ros2JointStateMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2JointStateMessageImpl();
    virtual ~Ros2JointStateMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Sets the joint state data
     * @param[in] timeStamp Time in seconds
     * @param[in] articulation Pointer to articulation view
     * @param[in] stage USD stage pointer
     * @param[in] jointPositions Vector of joint positions
     * @param[in] jointVelocities Vector of joint velocities
     * @param[in] jointEfforts Vector of joint efforts
     * @param[in] dofTypes Vector of DOF types
     * @param[in] stageUnits Stage unit scale factor
     */
    virtual void writeData(const double& timeStamp,
                           omni::physics::tensors::IArticulationView* articulation,
                           pxr::UsdStageWeakPtr stage,
                           std::vector<float>& jointPositions,
                           std::vector<float>& jointVelocities,
                           std::vector<float>& jointEfforts,
                           std::vector<uint8_t>& dofTypes,
                           const double& stageUnits);

    /**
     * @brief Reads the joint state data
     * @param[out] jointNames Vector of joint names
     * @param[out] jointPositions Array of joint positions
     * @param[out] jointVelocities Array of joint velocities
     * @param[out] jointEfforts Array of joint efforts
     * @param[out] timeStamp Time in seconds
     */
    virtual void readData(std::vector<char*>& jointNames,
                          double* jointPositions,
                          double* jointVelocities,
                          double* jointEfforts,
                          double& timeStamp);

    /**
     * @brief Gets the number of joints in the message
     * @return size_t Number of joints
     */
    virtual size_t getNumJoints();

    /**
     * @brief Validates the joint state message
     * @return bool True if the message is valid
     */
    virtual bool checkValid();
};

/**
 * @class Ros2PointCloudMessageImpl
 * @brief Implementation of ROS 2 Point Cloud message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/PointCloud2 messages,
 * which contain 3D point cloud data with customizable fields.
 */
class Ros2PointCloudMessageImpl : public Ros2PointCloudMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2PointCloudMessageImpl();
    virtual ~Ros2PointCloudMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Allocates and initializes the point cloud buffer
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the point cloud
     * @param[in] width Number of points in a row
     * @param[in] height Number of rows (1 for unorganized clouds)
     * @param[in] pointStep Size of a single point in bytes
     */
    virtual void generateBuffer(const double& timeStamp,
                                const std::string& frameId,
                                const size_t& width,
                                const size_t& height,
                                const uint32_t& pointStep);
};

/**
 * @class Ros2LaserScanMessageImpl
 * @brief Implementation of ROS 2 Laser Scan message
 * @details
 * Handles the creation and manipulation of sensor_msgs/msg/LaserScan messages,
 * which contain 2D laser range finder data.
 */
class Ros2LaserScanMessageImpl : public Ros2LaserScanMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2LaserScanMessageImpl();
    virtual ~Ros2LaserScanMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 LaserScan message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) override;

    /**
     * @brief Generate the buffers according to the LaserScan metadata.
     * @details
     * It allocates memory for the range and intensities data.
     *
     * @param[in] buffSize buffer size.
     */
    virtual void generateBuffers(const size_t buffSize) override;

    /**
     * @brief Sets the laser scan data
     * @param[in] azimuthRange Min and max angles in degrees
     * @param[in] rotationRate Scan frequency in Hz
     * @param[in] depthRange Min and max range values
     * @param[in] horizontalResolution Angular resolution in degrees
     * @param[in] horizontalFov Horizontal field of view in degrees
     */
    virtual void writeData(const pxr::GfVec2f& azimuthRange,
                           const float& rotationRate,
                           const pxr::GfVec2f& depthRange,
                           float horizontalResolution,
                           float horizontalFov) override;
};

/**
 * @class Ros2TfTreeMessageImpl
 * @brief Implementation of ROS 2 Transform Tree message
 * @details
 * Handles the creation and manipulation of tf2_msgs/msg/TFMessage messages,
 * which contain multiple coordinate frame transforms.
 */
class Ros2TfTreeMessageImpl : public Ros2TfTreeMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2TfTreeMessageImpl();
    virtual ~Ros2TfTreeMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Sets the transform tree data
     * @param[in] timeStamp Time in seconds
     * @param[in] transforms Vector of transform data
     */
    virtual void writeData(const double& timeStamp, std::vector<TfTransformStamped>& transforms);

    /**
     * @brief Reads the transform tree data
     * @param[out] transforms Vector to store transform data
     */
    virtual void readData(std::vector<TfTransformStamped>& transforms);
};

/**
 * @class Ros2TwistMessageImpl
 * @brief Implementation of ROS 2 Twist message
 * @details
 * Handles the creation and manipulation of geometry_msgs/msg/Twist messages,
 * which contain linear and angular velocity data.
 */
class Ros2TwistMessageImpl : public Ros2TwistMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2TwistMessageImpl();
    virtual ~Ros2TwistMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Reads the twist message data
     * @param[out] linearVelocity Linear velocity vector
     * @param[out] angularVelocity Angular velocity vector
     */
    virtual void readData(pxr::GfVec3d& linearVelocity, pxr::GfVec3d& angularVelocity);
};

/**
 * @class Ros2AckermannDriveStampedMessageImpl
 * @brief Implementation of ROS 2 Ackermann Drive Stamped message
 * @details
 * Handles the creation and manipulation of ackermann_msgs/msg/AckermannDriveStamped
 * messages, which contain Ackermann steering control commands.
 */
class Ros2AckermannDriveStampedMessageImpl : public Ros2AckermannDriveStampedMessage, Ros2MessageInterfaceImpl
{
public:
    Ros2AckermannDriveStampedMessageImpl();
    virtual ~Ros2AckermannDriveStampedMessageImpl();
    virtual const void* getTypeSupportHandle();
    virtual void readData(double& timeStamp,
                          std::string& frameId,
                          double& steeringAngle,
                          double& steeringAngleVelocity,
                          double& speed,
                          double& acceleration,
                          double& jerk);
    /**
     * @brief Writes the message header
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the command
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId);

    /**
     * @brief Sets the Ackermann drive data
     * @param[in] steeringAngle Steering angle in radians
     * @param[in] steeringAngleVelocity Rate of change of steering angle
     * @param[in] speed Forward speed
     * @param[in] acceleration Forward acceleration
     * @param[in] jerk Rate of change of acceleration
     */
    virtual void writeData(const double& steeringAngle,
                           const double& steeringAngleVelocity,
                           const double& speed,
                           const double& acceleration,
                           const double& jerk);
};

/**
 * @class Ros2ContextHandleImpl
 * @brief Implementation of ROS 2 Context Handle
 * @details
 * Manages the ROS 2 context lifecycle, including initialization,
 * shutdown, and state validation.
 */
class Ros2ContextHandleImpl : public Ros2ContextHandle
{
public:
    virtual ~Ros2ContextHandleImpl()
    {
        shutdown();
    }
    virtual void* getContext();
    virtual void init(int argc, char const* const* argv, bool setDomainId = false, size_t domainId = 0);
    virtual bool isValid();
    virtual bool shutdown(const char* shutdownReason = nullptr);

private:
    rcl_init_options_t m_initOptions;
    std::shared_ptr<rcl_context_t> m_context;
};

/**
 * @class Ros2NodeHandleImpl
 * @brief Implementation of ROS 2 Node Handle
 * @details
 * Manages a ROS 2 node instance, providing access to the node's
 * context and internal rcl_node_t structure.
 */
class Ros2NodeHandleImpl : public Ros2NodeHandle
{
public:
    /**
     * @brief Constructor for ROS 2 Node Handle implementation.
     * @param[in] name Name of the ROS 2 node.
     * @param[in] namespaceName Namespace for the node.
     * @param[in] contextHandle Pointer to the ROS 2 context handle.
     */
    Ros2NodeHandleImpl(const char* name, const char* namespaceName, Ros2ContextHandle* contextHandle);
    virtual ~Ros2NodeHandleImpl();
    virtual Ros2ContextHandle* getContextHandle();
    virtual void* getNode();

private:
    Ros2ContextHandle* m_contextHandle;
    std::shared_ptr<rcl_node_t> m_node;
};

/**
 * @class Ros2PublisherImpl
 * @brief Implementation of ROS 2 Publisher
 * @details
 * Manages a ROS 2 publisher instance, providing functionality
 * for publishing messages and monitoring subscriber count.
 */
class Ros2PublisherImpl : public Ros2Publisher
{
public:
    /**
     * @brief Constructor for ROS 2 Publisher implementation.
     * @param[in] nodeHandle Pointer to the ROS 2 node handle.
     * @param[in] topicName Name of the topic to publish on.
     * @param[in] typeSupport Type support handle for the message type.
     * @param[in] qos Quality of Service settings for the publisher.
     */
    Ros2PublisherImpl(Ros2NodeHandle* nodeHandle, const char* topicName, const void* typeSupport, const Ros2QoSProfile& qos);
    virtual ~Ros2PublisherImpl();
    virtual void publish(const void* msg);
    virtual size_t getSubscriptionCount();
    virtual bool isValid()
    {
        return m_publisher != nullptr;
    }

private:
    Ros2NodeHandle* m_nodeHandle;
    std::shared_ptr<rcl_publisher_t> m_publisher = nullptr;
};

/**
 * @class Ros2SubscriberImpl
 * @brief Implementation of ROS 2 Subscriber
 * @details
 * Manages a ROS 2 subscriber instance, providing functionality
 * for receiving and processing messages.
 */
class Ros2SubscriberImpl : public Ros2Subscriber
{
public:
    /**
     * @brief Constructor for ROS 2 Subscriber implementation.
     * @param[in] nodeHandle Pointer to the ROS 2 node handle.
     * @param[in] topicName Name of the topic to subscribe to.
     * @param[in] typeSupport Type support handle for the message type.
     * @param[in] qos Quality of Service settings for the subscriber.
     */
    Ros2SubscriberImpl(Ros2NodeHandle* nodeHandle,
                       const char* topicName,
                       const void* typeSupport,
                       const Ros2QoSProfile& qos);
    virtual ~Ros2SubscriberImpl();

    /**
     * @brief Processes incoming messages
     * @param[out] msg Pointer to store the received message
     * @return bool True if a message was received and processed
     */
    virtual bool spin(void* msg);

    /**
     * @brief Checks if the subscriber is valid
     * @return bool True if the subscriber is properly initialized
     */
    virtual bool isValid()
    {
        return m_subscription != nullptr;
    }

private:
    Ros2NodeHandle* m_nodeHandle;
    std::shared_ptr<rcl_subscription_t> m_subscription = nullptr;
    rcl_wait_set_t m_waitSet;
    bool m_waitSetInitialized = false;
};

/**
 * @class Ros2ServiceImpl
 * @brief Implementation of ROS 2 Service Server
 * @details
 * Manages a ROS 2 service server instance, providing functionality
 * for handling service requests and sending responses.
 */
class Ros2ServiceImpl : public Ros2Service
{
public:
    /**
     * @brief Constructor for ROS 2 Service Server implementation.
     * @param[in] nodeHandle Pointer to the ROS 2 node handle.
     * @param[in] serviceName Name of the service.
     * @param[in] typeSupport Type support handle for the service type.
     * @param[in] qos Quality of Service settings for the service.
     */
    Ros2ServiceImpl(Ros2NodeHandle* nodeHandle, const char* serviceName, const void* typeSupport, const Ros2QoSProfile& qos);
    virtual ~Ros2ServiceImpl();

    /**
     * @brief Takes a pending service request
     * @param[out] requestMsg Pointer to store the request message
     * @return bool True if a request was received
     */
    virtual bool takeRequest(void* requestMsg);

    /**
     * @brief Sends a service response
     * @param[in] responseMsg Pointer to the response message
     * @return bool True if the response was sent successfully
     */
    virtual bool sendResponse(void* responseMsg);

    /**
     * @brief Checks if the service server is valid
     * @return bool True if the service server is properly initialized
     */
    virtual bool isValid()
    {
        return m_service != nullptr;
    }

private:
    Ros2NodeHandle* m_nodeHandle;
    std::shared_ptr<rcl_service_t> m_service = nullptr;
    rcl_wait_set_t m_waitSet;
    rmw_request_id_t m_requestId;
    bool m_waitSetInitialized = false;
};

/**
 * @class Ros2ClientImpl
 * @brief Implementation of ROS 2 Service Client
 * @details
 * Manages a ROS 2 service client instance, providing functionality
 * for sending service requests and receiving responses.
 */
class Ros2ClientImpl : public Ros2Client
{
public:
    /**
     * @brief Constructor for ROS 2 Service Client implementation.
     * @param[in] nodeHandle Pointer to the ROS 2 node handle.
     * @param[in] serviceName Name of the service.
     * @param[in] typeSupport Type support handle for the service type.
     * @param[in] qos Quality of Service settings for the client.
     */
    Ros2ClientImpl(Ros2NodeHandle* nodeHandle, const char* serviceName, const void* typeSupport, const Ros2QoSProfile& qos);
    virtual ~Ros2ClientImpl();

    /**
     * @brief Sends a service request
     * @param[in] requestMsg Pointer to the request message
     * @return bool True if the request was sent successfully
     */
    virtual bool sendRequest(void* requestMsg);

    /**
     * @brief Takes a service response
     * @param[out] responseMsg Pointer to store the response message
     * @return bool True if a response was received
     */
    virtual bool takeResponse(void* responseMsg);

    /**
     * @brief Checks if the service client is valid
     * @return bool True if the service client is properly initialized
     */
    virtual bool isValid()
    {
        return m_client != nullptr;
    }

private:
    Ros2NodeHandle* m_nodeHandle;
    std::shared_ptr<rcl_client_t> m_client = nullptr;
    rcl_wait_set_t m_waitSet;
    rmw_request_id_t m_requestId;
    bool m_waitSetInitialized = false;
};

/**
 * @class Ros2DynamicMessageImpl
 * @brief Implementation of ROS 2 Dynamic Message
 * @details
 * Provides functionality for runtime message type handling,
 * including message field introspection, serialization,
 * and deserialization.
 */
class Ros2DynamicMessageImpl : public Ros2DynamicMessage, Ros2MessageInterfaceImpl
{
public:
    /**
     * @brief Constructor for ROS 2 Dynamic Message implementation.
     * @param[in] pkgName Name of the ROS 2 package containing the message definition.
     * @param[in] msgSubfolder Subfolder containing the message definition (e.g., "msg" or "srv").
     * @param[in] msgName Name of the message type.
     * @param[in] messageType Type of the message (default: eMessage).
     */
    Ros2DynamicMessageImpl(std::string pkgName,
                           std::string msgSubfolder,
                           std::string msgName,
                           BackendMessageType messageType = BackendMessageType::eMessage);
    virtual ~Ros2DynamicMessageImpl();
    virtual const void* getTypeSupportHandle();

    /**
     * @brief Generates a summary of the message structure
     * @param[in] print Whether to print the summary to console
     * @return std::string Formatted summary string
     */
    virtual std::string generateSummary(bool print);

    /**
     * @brief Reads message data as JSON
     * @return const nlohmann::json& Reference to JSON representation
     */
    virtual const nlohmann::json& readData();

    /**
     * @brief Reads message data as vector
     * @param[in] asOgnType Whether to convert to OmniGraph types
     * @return const std::vector<std::shared_ptr<void>>& Vector of field data
     */
    virtual const std::vector<std::shared_ptr<void>>& readData(bool asOgnType);

    /**
     * @brief Writes message data from JSON
     * @param[in] data JSON data to write
     */
    virtual void writeData(const nlohmann::json& data);

    /**
     * @brief Writes message data from vector
     * @param[in] data Vector of field data
     * @param[in] fromOgnType Whether data is in OmniGraph types
     */
    virtual void writeData(const std::vector<std::shared_ptr<void>>& data, bool fromOgnType);

protected:
    /**
     * @brief Gets the introspection members of the ROS2 message type.
     * @details
     * Retrieves the introspection members that describe the structure and layout
     * of the ROS2 message type. This is used for dynamic message handling and
     * type introspection.
     *
     * @return A pointer to the message members definition structure.
     */
    virtual const void* getIntrospectionMembers();
    /**
     * @brief Parses message field definitions
     * @param[in] parentName Name of the parent field
     * @param[in] members Pointer to message members definition
     */
    virtual void parseMessageFields(const std::string& parentName, const void* members);

    /**
     * @brief Gets message field values as JSON
     * @param[in] members Pointer to message members definition
     * @param[in] messageData Pointer to message data
     * @param[out] container JSON container to fill
     */
    virtual void getMessageValues(const void* members, uint8_t* messageData, nlohmann::json& container);

    /**
     * @brief Gets message field values as vector
     * @param[in] members Pointer to message members definition
     * @param[in] messageData Pointer to message data
     * @param[out] container Vector container to fill
     * @param[in,out] index Current field index
     * @param[in] asOgnType Whether to convert to OmniGraph types
     */
    virtual void getMessageValues(const void* members,
                                  uint8_t* messageData,
                                  std::vector<std::shared_ptr<void>>& container,
                                  size_t& index,
                                  bool asOgnType);

    /**
     * @brief Sets message field values from JSON
     * @param[in] members Pointer to message members definition
     * @param[out] messageData Pointer to message data
     * @param[in] container JSON container with field values
     */
    virtual void setMessageValues(const void* members, uint8_t* messageData, const nlohmann::json& container);

    /**
     * @brief Sets message field values from vector
     * @param[in] members Pointer to message members definition
     * @param[out] messageData Pointer to message data
     * @param[in] container Vector container with field values
     * @param[in,out] index Current field index
     * @param[in] fromOgnType Whether data is in OmniGraph types
     */
    virtual void setMessageValues(const void* members,
                                  uint8_t* messageData,
                                  const std::vector<std::shared_ptr<void>>& container,
                                  size_t& index,
                                  bool fromOgnType);

    /**
     * @brief Gets array values from ROS message and converts them to JSON format
     * @details
     * Template function that extracts array data from a ROS message and stores it in a JSON array.
     *
     * @tparam ArrayType The ROS array type (e.g., rosidl_runtime_c__float__Sequence)
     * @tparam RosType The underlying ROS data type (e.g., float)
     *
     * @param[in] member Pointer to the message member definition
     * @param[in] data Pointer to the message data
     * @param[out] array JSON array to store the extracted values
     */
    template <typename ArrayType, typename RosType>
    void getArray(const rosidl_typesupport_introspection_c__MessageMember* member, uint8_t* data, nlohmann::json& array);

    /**
     * @brief Gets array values from ROS message and converts them to OmniGraph types
     * @details
     * Template function that extracts array data from a ROS message and converts it to OmniGraph format.
     *
     * @tparam ArrayType The ROS array type
     * @tparam RosType The underlying ROS data type
     * @tparam OgnType The target OmniGraph data type
     *
     * @param[in] member Pointer to the message member definition
     * @param[in] data Pointer to the message data
     * @param[out] valuePtr Shared pointer to store the converted data
     * @param[in] asOgnType Whether to convert to OmniGraph type
     */
    template <typename ArrayType, typename RosType, typename OgnType>
    void getArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                  uint8_t* data,
                  std::shared_ptr<void>& valuePtr,
                  bool asOgnType);

    /**
     * @brief Gets array values from ROS message into a vector
     * @details
     * Template function that extracts array data from a ROS message into a std::vector.
     *
     * @tparam ArrayType The ROS array type
     * @tparam RosType The underlying ROS data type
     *
     * @param[in] member Pointer to the message member definition
     * @param[in] data Pointer to the message data
     * @param[out] array Vector to store the extracted values
     */
    template <typename ArrayType, typename RosType>
    void getArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                  uint8_t* data,
                  std::vector<RosType>& array);

    /**
     * @brief Sets array values in ROS message from JSON data
     * @details
     * Template function that populates a ROS message array from JSON data.
     *
     * @tparam ArrayType The ROS array type
     * @tparam ArrayInit Function pointer to array initialization function
     * @tparam RosType The underlying ROS data type
     *
     * @param[in] member Pointer to the message member definition
     * @param[out] data Pointer to the message data
     * @param[in] value JSON value containing the array data
     */
    template <typename ArrayType, auto ArrayInit, typename RosType>
    void setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                  uint8_t* data,
                  const nlohmann::json& value);

    /**
     * @brief Sets array values in ROS message from OmniGraph data
     * @details
     * Template function that populates a ROS message array from OmniGraph data.
     *
     * @tparam ArrayType The ROS array type
     * @tparam ArrayInit Function pointer to array initialization function
     * @tparam RosType The underlying ROS data type
     * @tparam OgnType The source OmniGraph data type
     *
     * @param[in] member Pointer to the message member definition
     * @param[out] data Pointer to the message data
     * @param[in] valuePtr Shared pointer to the source data
     * @param[in] fromOgnType Whether the source is in OmniGraph format
     */
    template <typename ArrayType, auto ArrayInit, typename RosType, typename OgnType>
    void setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                  uint8_t* data,
                  const std::shared_ptr<void>& valuePtr,
                  bool fromOgnType);

    /**
     * @brief Sets array values in ROS message from vector data
     * @details
     * Template function that populates a ROS message array from a std::vector.
     *
     * @tparam ArrayType The ROS array type
     * @tparam ArrayInit Function pointer to array initialization function
     * @tparam RosType The underlying ROS data type
     *
     * @param[in] member Pointer to the message member definition
     * @param[out] data Pointer to the message data
     * @param[in] array Vector containing the source data
     */
    template <typename ArrayType, auto ArrayInit, typename RosType>
    void setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                  uint8_t* data,
                  const std::vector<RosType>& array);

    /**
     * @brief Gets a single value from ROS message and converts to OmniGraph type
     * @details
     * Template function that extracts a single value from a ROS message and optionally converts it
     * to OmniGraph format.
     *
     * @tparam RosType The ROS data type
     * @tparam OgnType The target OmniGraph data type
     *
     * @param[in] data Pointer to the message data
     * @param[out] valuePtr Shared pointer to store the converted value
     * @param[in] asOgnType Whether to convert to OmniGraph type
     */
    template <typename RosType, typename OgnType>
    void getSingleValue(uint8_t* data, std::shared_ptr<void>& valuePtr, bool asOgnType);

    /**
     * @brief Sets a single value in ROS message from OmniGraph data
     * @details
     * Template function that sets a single value in a ROS message, optionally converting from
     * OmniGraph format.
     *
     * @tparam RosType The ROS data type
     * @tparam OgnType The source OmniGraph data type
     *
     * @param[out] data Pointer to the message data
     * @param[in] valuePtr Shared pointer to the source value
     * @param[in] fromOgnType Whether the source is in OmniGraph format
     */
    template <typename RosType, typename OgnType>
    void setSingleValue(uint8_t* data, const std::shared_ptr<void>& valuePtr, bool fromOgnType);

    /**
     * @brief Gets array of embedded messages as JSON
     * @details
     * Extracts an array of embedded messages from a ROS message and converts them to JSON format.
     * This function handles complex message types that contain nested message structures.
     *
     * @param[in] member Pointer to the message member definition containing the array
     * @param[in] data Pointer to the message data
     * @param[out] array JSON array to store the extracted message data
     */
    void getArrayEmbeddedMessage(const rosidl_typesupport_introspection_c__MessageMember* member,
                                 uint8_t* data,
                                 nlohmann::json& array);

    /**
     * @brief Sets array of embedded messages from JSON
     * @details
     * Populates an array of embedded messages in a ROS message from JSON data.
     * This function handles complex message types that contain nested message structures.
     *
     * @param[in] member Pointer to the message member definition for the array
     * @param[out] data Pointer to the message data to be populated
     * @param[in] array JSON array containing the source message data
     */
    void setArrayEmbeddedMessage(const rosidl_typesupport_introspection_c__MessageMember* member,
                                 uint8_t* data,
                                 const nlohmann::json& array);
};

/**
 * @class Ros2QoSProfileConverter
 * @brief Utility class for converting QoS profiles between formats
 * @details
 * Provides functionality to convert between different Quality of Service (QoS) profile
 * representations used in ROS 2 communication. This class helps ensure proper QoS
 * settings are maintained across different parts of the system.
 */
class Ros2QoSProfileConverter
{
public:
    /**
     * @brief Converts a QoS profile to RMW format
     * @details
     * Converts a ROS 2 QoS profile from the bridge's internal representation to the
     * ROS Middleware (RMW) format used by the ROS 2 implementation.
     *
     * @param[in] qos The QoS profile to convert
     * @return rmw_qos_profile_t The converted QoS profile in RMW format
     */
    static rmw_qos_profile_t convert(const Ros2QoSProfile& qos);
};

} // namespace bridge
} // namespace ros2
} // namespace isaacsim
