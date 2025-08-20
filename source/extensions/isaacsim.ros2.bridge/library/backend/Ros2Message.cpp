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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on
#include "Ros2Impl.h"
#include "isaacsim/core/includes/UsdUtilities.h"
#include "pxr/usd/usdPhysics/joint.h"
#include "sensor_msgs/image_encodings.hpp"

#include <carb/logging/Log.h>

#include <rcl/rcl.h>
#include <sensor_msgs/msg/camera_info.h>

namespace isaacsim
{
namespace ros2
{
namespace bridge
{

// Clock message
Ros2ClockMessageImpl::Ros2ClockMessageImpl() : Ros2MessageInterfaceImpl("rosgraph_msgs", "msg", "Clock")
{
    m_msg = rosgraph_msgs__msg__Clock__create();
}

Ros2ClockMessageImpl::~Ros2ClockMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    rosgraph_msgs__msg__Clock__destroy(static_cast<rosgraph_msgs__msg__Clock*>(m_msg));
}

const void* Ros2ClockMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(rosgraph_msgs, msg, Clock);
}

void Ros2ClockMessageImpl::writeData(double timeStamp)
{
    if (!m_msg)
    {
        return;
    }
    rosgraph_msgs__msg__Clock* clockMsg = static_cast<rosgraph_msgs__msg__Clock*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosTime(static_cast<int64_t>(timeStamp * 1e9), clockMsg->clock);
}

void Ros2ClockMessageImpl::readData(double& timeStamp)
{
    if (!m_msg)
    {
        return;
    }
    rosgraph_msgs__msg__Clock* clockMsg = static_cast<rosgraph_msgs__msg__Clock*>(m_msg);
    timeStamp = clockMsg->clock.sec + clockMsg->clock.nanosec / 1e9;
}

// Imu message
Ros2ImuMessageImpl::Ros2ImuMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "Imu")
{
    m_msg = sensor_msgs__msg__Imu__create();
}

const void* Ros2ImuMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
}

void Ros2ImuMessageImpl::writeHeader(double timeStamp, std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Imu* imuMsg = static_cast<sensor_msgs__msg__Imu*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), imuMsg->header);
}

void Ros2ImuMessageImpl::writeAcceleration(bool covariance = false,
                                           const std::vector<double>& acceleration = std::vector<double>())
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Imu* imuMsg = static_cast<sensor_msgs__msg__Imu*>(m_msg);
    if (covariance)
    {
        imuMsg->linear_acceleration_covariance[0] = -1;
    }
    else
    {
        imuMsg->linear_acceleration.x = acceleration[0];
        imuMsg->linear_acceleration.y = acceleration[1];
        imuMsg->linear_acceleration.z = acceleration[2];
    }
}

void Ros2ImuMessageImpl::writeVelocity(bool covariance = false,
                                       const std::vector<double>& velocity = std::vector<double>())
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Imu* imuMsg = static_cast<sensor_msgs__msg__Imu*>(m_msg);
    if (covariance)
    {
        imuMsg->angular_velocity_covariance[0] = -1;
    }
    else
    {
        imuMsg->angular_velocity.x = velocity[0];
        imuMsg->angular_velocity.y = velocity[1];
        imuMsg->angular_velocity.z = velocity[2];
    }
}

void Ros2ImuMessageImpl::writeOrientation(bool covariance = false,
                                          const std::vector<double>& orientation = std::vector<double>())
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Imu* imuMsg = static_cast<sensor_msgs__msg__Imu*>(m_msg);
    if (covariance)
    {
        imuMsg->orientation_covariance[0] = -1;
    }
    else
    {
        imuMsg->orientation.x = orientation[0];
        imuMsg->orientation.y = orientation[1];
        imuMsg->orientation.z = orientation[2];
        imuMsg->orientation.w = orientation[3];
    }
}

Ros2ImuMessageImpl::~Ros2ImuMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Imu__destroy(static_cast<sensor_msgs__msg__Imu*>(m_msg));
}

// CameraInfo message
Ros2CameraInfoMessageImpl::Ros2CameraInfoMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "CameraInfo")
{
    m_msg = sensor_msgs__msg__CameraInfo__create();
    if (!m_msg)
    {
        CARB_LOG_ERROR("Failed to create sensor_msgs CameraInfo message");
    }
}

const void* Ros2CameraInfoMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CameraInfo);
}

void Ros2CameraInfoMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), cameraInfoMsg->header);
}

void Ros2CameraInfoMessageImpl::writeResolution(const uint32_t height, const uint32_t width)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);
    cameraInfoMsg->height = height;
    cameraInfoMsg->width = width;
}

void Ros2CameraInfoMessageImpl::writeIntrinsicMatrix(const double array[], const size_t arraySize)
{
    if (!m_msg)
    {
        return;
    }

    // Validate input parameters
    if (!array)
    {
        CARB_LOG_ERROR("writeIntrinsicMatrix: input array is null");
        return;
    }

    if (arraySize != 9)
    {
        CARB_LOG_ERROR("writeIntrinsicMatrix: invalid array size %zu, expected 9 for 3x3 matrix", arraySize);
        return;
    }

    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);

    // The k field in sensor_msgs CameraInfo is a fixed-size array of 9 doubles
    memcpy(cameraInfoMsg->k, array, arraySize * sizeof(double));
}

void Ros2CameraInfoMessageImpl::writeDistortionParameters(std::vector<double>& array, const std::string& distortionModel)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);
    if (!array.empty())
    {
        m_distortionBuffer = array; // Copy the data to our member vector
        cameraInfoMsg->d.data = m_distortionBuffer.data();
        cameraInfoMsg->d.size = m_distortionBuffer.size();
        cameraInfoMsg->d.capacity = m_distortionBuffer.size();
    }
    Ros2MessageInterfaceImpl::writeRosString(distortionModel, cameraInfoMsg->distortion_model);
}

void Ros2CameraInfoMessageImpl::writeProjectionMatrix(const double array[], const size_t arraySize)
{
    if (!m_msg)
    {
        return;
    }

    // Validate input parameters
    if (!array)
    {
        CARB_LOG_ERROR("writeProjectionMatrix: input array is null");
        return;
    }

    if (arraySize != 12)
    {
        CARB_LOG_ERROR("writeProjectionMatrix: invalid array size %zu, expected 12 for 3x4 matrix", arraySize);
        return;
    }

    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);

    // The p field in sensor_msgs CameraInfo is a fixed-size array of 12 doubles
    memcpy(cameraInfoMsg->p, array, arraySize * sizeof(double));
}

void Ros2CameraInfoMessageImpl::writeRectificationMatrix(const double array[], const size_t arraySize)
{
    if (!m_msg)
    {
        return;
    }

    // Validate input parameters
    if (!array)
    {
        CARB_LOG_ERROR("writeRectificationMatrix: input array is null");
        return;
    }

    if (arraySize != 9)
    {
        CARB_LOG_ERROR("writeRectificationMatrix: invalid array size %zu, expected 9 for 3x3 matrix", arraySize);
        return;
    }

    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);

    // The r field in sensor_msgs CameraInfo is a fixed-size array of 9 doubles
    memcpy(cameraInfoMsg->r, array, arraySize * sizeof(double));
}

Ros2CameraInfoMessageImpl::~Ros2CameraInfoMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__CameraInfo* cameraInfoMsg = static_cast<sensor_msgs__msg__CameraInfo*>(m_msg);

    // Clear the pointer but don't free - memory is managed by m_distortionBuffer
    cameraInfoMsg->d.data = nullptr;
    cameraInfoMsg->d.size = 0;
    cameraInfoMsg->d.capacity = 0;

    sensor_msgs__msg__CameraInfo__destroy(cameraInfoMsg);
}

// Image message
Ros2ImageMessageImpl::Ros2ImageMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "Image")
{
    m_msg = sensor_msgs__msg__Image__create();
}

const void* Ros2ImageMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image);
}

void Ros2ImageMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Image* imageMsg = static_cast<sensor_msgs__msg__Image*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), imageMsg->header);
}

void Ros2ImageMessageImpl::generateBuffer(const uint32_t height, const uint32_t width, const std::string& encoding)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Image* imageMsg = static_cast<sensor_msgs__msg__Image*>(m_msg);
    imageMsg->height = height;
    imageMsg->width = width;
    Ros2MessageInterfaceImpl::writeRosString(encoding, imageMsg->encoding);

    int channels = 0;
    int bitDepth = 0;
    try
    {
        channels = sensor_msgs::image_encodings::numChannels(encoding);
        bitDepth = sensor_msgs::image_encodings::bitDepth(encoding);
    }
    catch (std::exception& e)
    {
        fprintf(stderr, "[Error] %s\n", e.what());
        return;
    }
    int byteDepth = bitDepth / 8;

    uint32_t step = width * channels * byteDepth;
    imageMsg->step = step;
    m_totalBytes = step * height;
    m_buffer.resize(m_totalBytes);
    imageMsg->data.size = m_totalBytes;
    imageMsg->data.capacity = m_totalBytes;
    imageMsg->data.data = &m_buffer[0];
}

Ros2ImageMessageImpl::~Ros2ImageMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__Image* imageMsg = static_cast<sensor_msgs__msg__Image*>(m_msg);
    // Lifetime of memory is not managed by the message as we use a std vector
    imageMsg->data.size = 0;
    imageMsg->data.capacity = 0;
    imageMsg->data.data = nullptr;
    sensor_msgs__msg__Image__destroy(imageMsg);
}

// NitrosBridgeImage message
// For this specific message we disable logging when loading the message library to prevent spam
Ros2NitrosBridgeImageMessageImpl::Ros2NitrosBridgeImageMessageImpl()
    : Ros2MessageInterfaceImpl(
          "isaac_ros_nitros_bridge_interfaces", "msg", "NitrosBridgeImage", BackendMessageType::eMessage, true)
{
#if !defined(_WIN32)
    if (m_typesupportLibrary->isValid())
    {
        m_msg = create();
    }
#endif
}

const void* Ros2NitrosBridgeImageMessageImpl::getTypeSupportHandle()
{
    return getTypeSupportHandleDynamic();
}

void Ros2NitrosBridgeImageMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
#if !defined(_WIN32)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage* imageMsg =
        static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), imageMsg->header);
#endif
}

void Ros2NitrosBridgeImageMessageImpl::generateBuffer(const uint32_t height,
                                                      const uint32_t width,
                                                      const std::string& encoding)
{
    if (!m_msg)
    {
        return;
    }
#if !defined(_WIN32)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage* imageMsg =
        static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg);
    imageMsg->height = height;
    imageMsg->width = width;
    Ros2MessageInterfaceImpl::writeRosString(encoding, imageMsg->encoding);

    int channels = 0;
    int bitDepth = 0;
    try
    {
        channels = sensor_msgs::image_encodings::numChannels(encoding);
        bitDepth = sensor_msgs::image_encodings::bitDepth(encoding);
    }
    catch (std::exception& e)
    {
        fprintf(stderr, "[Error] %s\n", e.what());
        return;
    }
    int byteDepth = bitDepth / 8;

    uint32_t step = width * channels * byteDepth;
    imageMsg->step = step;
    m_totalBytes = step * height;
#endif
}

void Ros2NitrosBridgeImageMessageImpl::writeData(const std::vector<int32_t>& data)
{
    if (!m_msg || data.empty())
    {
        return;
    }
#if !defined(_WIN32)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage* imageMsg =
        static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg);

    m_imageData.resize(data.size());
    std::memcpy(m_imageData.data(), data.data(), data.size() * sizeof(int32_t));

    imageMsg->data.size = m_imageData.size();
    imageMsg->data.capacity = m_imageData.size();
    imageMsg->data.data = &m_imageData[0];
#endif
}

Ros2NitrosBridgeImageMessageImpl::~Ros2NitrosBridgeImageMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
#if !defined(_WIN32)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage* imageMsg =
        static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg);

    // Lifetime of memory is not managed by the message as we use a std vector
    imageMsg->data.size = 0;
    imageMsg->data.capacity = 0;
    imageMsg->data.data = nullptr;
    destroy(static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg));
#endif
}

// 2D bounding box detection message
struct Bbox2DData
{
    uint32_t semanticId;
    int32_t xMin;
    int32_t yMin;
    int32_t xMax;
    int32_t yMax;
    float occlusionRatio;
};

Ros2BoundingBox2DMessageImpl::Ros2BoundingBox2DMessageImpl()
    : Ros2MessageInterfaceImpl("vision_msgs", "msg", "Detection2DArray")
{
    m_msg = create();
}

const void* Ros2BoundingBox2DMessageImpl::getTypeSupportHandle()
{
    return getTypeSupportHandleDynamic();
}

void Ros2BoundingBox2DMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    vision_msgs__msg__Detection2DArray* detectionMsg = static_cast<vision_msgs__msg__Detection2DArray*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), detectionMsg->header);
}

void Ros2BoundingBox2DMessageImpl::writeBboxData(const void* bboxArray, const size_t numBoxes)
{
    if (!m_msg)
    {
        return;
    }
    vision_msgs__msg__Detection2DArray* detectionMsg = static_cast<vision_msgs__msg__Detection2DArray*>(m_msg);

    // Set the detection sequence size and object pose sequence size
    m_generatorLibrary->callSymbolWithArg<void>(
        "vision_msgs__msg__Detection2D__Sequence__init", &detectionMsg->detections, numBoxes);

    const Bbox2DData* bboxData = reinterpret_cast<const Bbox2DData*>(bboxArray);

    for (size_t i = 0; i < numBoxes; i++)
    {
        const Bbox2DData& box = bboxData[i];

        detectionMsg->detections.data[i].bbox.center.theta = 0;
        detectionMsg->detections.data[i].bbox.center.position.x = (box.xMax + box.xMin) / 2.0;
        detectionMsg->detections.data[i].bbox.center.position.y = (box.yMax + box.yMin) / 2.0;
        detectionMsg->detections.data[i].bbox.size_x = box.xMax - box.xMin;
        detectionMsg->detections.data[i].bbox.size_y = box.yMax - box.yMin;
        // TODO: Detection sub message header for all detections
        // detectionMsg->detections.data[i].header

        m_generatorLibrary->callSymbolWithArg<void>(
            "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__init", &detectionMsg->detections.data[i].results, 1);

        detectionMsg->detections.data[i].results.data[0].hypothesis.score = 1.0;
        Ros2MessageInterfaceImpl::writeRosString(
            std::to_string(box.semanticId), detectionMsg->detections.data[i].results.data[0].hypothesis.class_id);
    }
}

Ros2BoundingBox2DMessageImpl::~Ros2BoundingBox2DMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    destroy(static_cast<vision_msgs__msg__Detection2DArray*>(m_msg));
}

// 3D bounding box detection message
struct Bbox3DData
{
    uint32_t semanticId;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;
    pxr::GfMatrix4f transform;
    float occlusionRatio;
};

Ros2BoundingBox3DMessageImpl::Ros2BoundingBox3DMessageImpl()
    : Ros2MessageInterfaceImpl("vision_msgs", "msg", "Detection3DArray")
{
    m_msg = create();
}

const void* Ros2BoundingBox3DMessageImpl::getTypeSupportHandle()
{
    return getTypeSupportHandleDynamic();
}

void Ros2BoundingBox3DMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    vision_msgs__msg__Detection3DArray* detectionMsg = static_cast<vision_msgs__msg__Detection3DArray*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), detectionMsg->header);
}

void Ros2BoundingBox3DMessageImpl::writeBboxData(const void* bboxArray, size_t numBoxes)
{
    if (!m_msg)
    {
        return;
    }
    vision_msgs__msg__Detection3DArray* detectionMsg = static_cast<vision_msgs__msg__Detection3DArray*>(m_msg);

    m_generatorLibrary->callSymbolWithArg<void>(
        "vision_msgs__msg__Detection3D__Sequence__init", &detectionMsg->detections, numBoxes);

    const Bbox3DData* bboxData = reinterpret_cast<const Bbox3DData*>(bboxArray);

    for (size_t i = 0; i < numBoxes; i++)
    {
        const Bbox3DData& box = bboxData[i];
        auto mat = pxr::GfMatrix4d(box.transform);
        auto transform = pxr::GfTransform(mat);

        auto trans = transform.GetTranslation();
        auto rot = transform.GetRotation().GetQuaternion();
        auto scale = transform.GetScale();

        // TODO: Detection sub message header for all detections
        // detectionMsg->detections.data[i].header

        detectionMsg->detections.data[i].bbox.center.position.x = trans[0];
        detectionMsg->detections.data[i].bbox.center.position.y = trans[1];
        detectionMsg->detections.data[i].bbox.center.position.z = trans[2];

        auto imag = rot.GetImaginary();

        detectionMsg->detections.data[i].bbox.center.orientation.x = imag[0];
        detectionMsg->detections.data[i].bbox.center.orientation.y = imag[1];
        detectionMsg->detections.data[i].bbox.center.orientation.z = imag[2];
        detectionMsg->detections.data[i].bbox.center.orientation.w = rot.GetReal();

        detectionMsg->detections.data[i].bbox.size.x = (box.xMax - box.xMin) * scale[0];
        detectionMsg->detections.data[i].bbox.size.y = (box.yMax - box.yMin) * scale[1];
        detectionMsg->detections.data[i].bbox.size.z = (box.zMax - box.zMin) * scale[2];


        m_generatorLibrary->callSymbolWithArg<void>(
            "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__init", &detectionMsg->detections.data[i].results, 1);

        detectionMsg->detections.data[i].results.data[0].hypothesis.score = 1.0;
        Ros2MessageInterfaceImpl::writeRosString(
            std::to_string(box.semanticId), detectionMsg->detections.data[i].results.data[0].hypothesis.class_id);
    }
}

Ros2BoundingBox3DMessageImpl::~Ros2BoundingBox3DMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    destroy(static_cast<vision_msgs__msg__Detection3DArray*>(m_msg));
}

// Odometry message
Ros2OdometryMessageImpl::Ros2OdometryMessageImpl() : Ros2MessageInterfaceImpl("nav_msgs", "msg", "Odometry")
{
    m_msg = nav_msgs__msg__Odometry__create();
}

const void* Ros2OdometryMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
}

void Ros2OdometryMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    nav_msgs__msg__Odometry* odometryMsg = static_cast<nav_msgs__msg__Odometry*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), odometryMsg->header);
}

void Ros2OdometryMessageImpl::writeData(std::string& childFrame,
                                        const pxr::GfVec3d& linearVelocity,
                                        const pxr::GfVec3d& angularVelocity,
                                        const pxr::GfVec3d& robotFront,
                                        const pxr::GfVec3d& robotSide,
                                        const pxr::GfVec3d& robotUp,
                                        double unitScale,
                                        const pxr::GfVec3d& position,
                                        const pxr::GfQuatd& orientation,
                                        bool publishRawVelocities)
{
    if (!m_msg)
    {
        return;
    }
    nav_msgs__msg__Odometry* odometryMsg = static_cast<nav_msgs__msg__Odometry*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosString(childFrame, odometryMsg->child_frame_id);

    if (publishRawVelocities)
    {
        // Directly set the velocities without projection
        odometryMsg->twist.twist.linear.x = linearVelocity[0] * unitScale;
        odometryMsg->twist.twist.linear.y = linearVelocity[1] * unitScale;
        odometryMsg->twist.twist.linear.z = linearVelocity[2] * unitScale;

        odometryMsg->twist.twist.angular.x = angularVelocity[0];
        odometryMsg->twist.twist.angular.y = angularVelocity[1];
        odometryMsg->twist.twist.angular.z = angularVelocity[2];
    }

    else
    {
        // Project robot velocities into robot frame using dot-products
        odometryMsg->twist.twist.linear.x = pxr::GfDot(linearVelocity, robotFront) * unitScale;
        odometryMsg->twist.twist.linear.y = pxr::GfDot(linearVelocity, robotSide) * unitScale;
        odometryMsg->twist.twist.linear.z = pxr::GfDot(linearVelocity, robotUp) * unitScale;

        odometryMsg->twist.twist.angular.x = pxr::GfDot(angularVelocity, robotFront);
        odometryMsg->twist.twist.angular.y = pxr::GfDot(angularVelocity, robotSide);
        odometryMsg->twist.twist.angular.z = pxr::GfDot(angularVelocity, robotUp);
    }
    odometryMsg->pose.pose.position.x = position[0];
    odometryMsg->pose.pose.position.y = position[1];
    odometryMsg->pose.pose.position.z = position[2];

    odometryMsg->pose.pose.orientation.x = orientation.GetImaginary()[0];
    odometryMsg->pose.pose.orientation.y = orientation.GetImaginary()[1];
    odometryMsg->pose.pose.orientation.z = orientation.GetImaginary()[2];
    odometryMsg->pose.pose.orientation.w = orientation.GetReal();
}

Ros2OdometryMessageImpl::~Ros2OdometryMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    nav_msgs__msg__Odometry__destroy(static_cast<nav_msgs__msg__Odometry*>(m_msg));
}

// Raw Tf tree message
Ros2RawTfTreeMessageImpl::Ros2RawTfTreeMessageImpl() : Ros2MessageInterfaceImpl("tf2_msgs", "msg", "TFMessage")
{
    m_msg = tf2_msgs__msg__TFMessage__create();
}

const void* Ros2RawTfTreeMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage);
}

void Ros2RawTfTreeMessageImpl::writeData(const double timeStamp,
                                         const std::string& frameId,
                                         const std::string& childFrame,
                                         const pxr::GfVec3d& translation,
                                         const pxr::GfQuatd& rotation)
{
    if (!m_msg)
    {
        return;
    }
    tf2_msgs__msg__TFMessage* tfMsg = static_cast<tf2_msgs__msg__TFMessage*>(m_msg);
    geometry_msgs__msg__TransformStamped__Sequence__init(&tfMsg->transforms, 1);

    Ros2MessageInterfaceImpl::writeRosHeader(
        frameId, static_cast<int64_t>(timeStamp * 1e9), tfMsg->transforms.data->header);
    Ros2MessageInterfaceImpl::writeRosString(childFrame, tfMsg->transforms.data->child_frame_id);

    tfMsg->transforms.data->transform.translation.x = translation[0];
    tfMsg->transforms.data->transform.translation.y = translation[1];
    tfMsg->transforms.data->transform.translation.z = translation[2];

    tfMsg->transforms.data->transform.rotation.x = rotation.GetImaginary()[0];
    tfMsg->transforms.data->transform.rotation.y = rotation.GetImaginary()[1];
    tfMsg->transforms.data->transform.rotation.z = rotation.GetImaginary()[2];
    tfMsg->transforms.data->transform.rotation.w = rotation.GetReal();
}

Ros2RawTfTreeMessageImpl::~Ros2RawTfTreeMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    tf2_msgs__msg__TFMessage__destroy(static_cast<tf2_msgs__msg__TFMessage*>(m_msg));
}

// Sematic label message (string type message)
Ros2SemanticLabelMessageImpl::Ros2SemanticLabelMessageImpl() : Ros2MessageInterfaceImpl("std_msgs", "msg", "String")
{
    m_msg = std_msgs__msg__String__create();
}

const void* Ros2SemanticLabelMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
}

void Ros2SemanticLabelMessageImpl::writeData(const std::string& data)
{
    if (!m_msg)
    {
        return;
    }
    std_msgs__msg__String* stringMsg = static_cast<std_msgs__msg__String*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosString(data, stringMsg->data);
}

Ros2SemanticLabelMessageImpl::~Ros2SemanticLabelMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    std_msgs__msg__String__destroy(static_cast<std_msgs__msg__String*>(m_msg));
}

// JointState message
Ros2JointStateMessageImpl::Ros2JointStateMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "JointState")
{
    m_msg = sensor_msgs__msg__JointState__create();
}

const void* Ros2JointStateMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
}

template <typename T>
static void createTensorDesc(omni::physics::tensors::TensorDesc& tensorDesc,
                             std::vector<T>& buffer,
                             int numElements,
                             omni::physics::tensors::TensorDataType type)
{
    buffer.resize(numElements);
    tensorDesc.dtype = type;
    tensorDesc.numDims = 1;
    tensorDesc.dims[0] = numElements;
    tensorDesc.data = buffer.data();
    tensorDesc.ownData = true;
    tensorDesc.device = -1;
}

void Ros2JointStateMessageImpl::writeData(const double& timeStamp,
                                          omni::physics::tensors::IArticulationView* articulation,
                                          pxr::UsdStageWeakPtr stage,
                                          std::vector<float>& jointPositions,
                                          std::vector<float>& jointVelocities,
                                          std::vector<float>& jointEfforts,
                                          std::vector<uint8_t>& dofTypes,
                                          const double& stageUnits)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__JointState* jointStateMsg = static_cast<sensor_msgs__msg__JointState*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader("", static_cast<int64_t>(timeStamp * 1e9), jointStateMsg->header);

    uint32_t numDofs = articulation->getMaxDofs();
    omni::physics::tensors::TensorDesc positionTensor;
    omni::physics::tensors::TensorDesc velocityTensor;
    omni::physics::tensors::TensorDesc effortTensor;
    omni::physics::tensors::TensorDesc dofTypeTensor;
    createTensorDesc(positionTensor, jointPositions, numDofs, omni::physics::tensors::TensorDataType::eFloat32);
    createTensorDesc(velocityTensor, jointVelocities, numDofs, omni::physics::tensors::TensorDataType::eFloat32);
    createTensorDesc(effortTensor, jointEfforts, numDofs, omni::physics::tensors::TensorDataType::eFloat32);
    createTensorDesc(dofTypeTensor, dofTypes, numDofs, omni::physics::tensors::TensorDataType::eUint8);
    bool hasDofStates = true;
    if (!articulation->getDofPositions(&positionTensor))
    {
        printf("Failed to get dof positions\n");
        hasDofStates = false;
    }
    if (!articulation->getDofVelocities(&velocityTensor))
    {
        printf("Failed to get dof velocities\n");
        hasDofStates = false;
    }
    if (!articulation->getDofProjectedJointForces(&effortTensor))
    {
        printf("Failed to get dof efforts\n");
        hasDofStates = false;
    }
    if (!articulation->getDofTypes(&dofTypeTensor))
    {
        printf("Failed to get dof types\n");
        hasDofStates = false;
    }

    rosidl_runtime_c__String__Sequence__init(&jointStateMsg->name, numDofs);
    rosidl_runtime_c__double__Sequence__init(&jointStateMsg->position, numDofs);
    rosidl_runtime_c__double__Sequence__init(&jointStateMsg->velocity, numDofs);
    rosidl_runtime_c__double__Sequence__init(&jointStateMsg->effort, numDofs);

    if (hasDofStates)
    {
        for (uint32_t j = 0; j < numDofs; j++)
        {
            const char* jointPath = articulation->getUsdDofPath(0, j);
            if (jointPath)
            {
                Ros2MessageInterfaceImpl::writeRosString(
                    isaacsim::core::includes::getName(stage->GetPrimAtPath(pxr::SdfPath(jointPath))),
                    jointStateMsg->name.data[j]);
            }
            if (static_cast<omni::physics::tensors::DofType>(dofTypes[j]) == omni::physics::tensors::DofType::eTranslation)
            {
                jointStateMsg->position.data[j] =
                    isaacsim::core::includes::math::roundNearest(jointPositions[j] * stageUnits, 10000.0); // m
                jointStateMsg->velocity.data[j] =
                    isaacsim::core::includes::math::roundNearest(jointVelocities[j] * stageUnits, 10000.0); // m/s
                jointStateMsg->effort.data[j] =
                    isaacsim::core::includes::math::roundNearest(jointEfforts[j] * stageUnits, 10000.0); // N
            }
            else
            {
                jointStateMsg->position.data[j] =
                    isaacsim::core::includes::math::roundNearest(jointPositions[j], 10000.0); // rad
                jointStateMsg->velocity.data[j] =
                    isaacsim::core::includes::math::roundNearest(jointVelocities[j], 10000.0); // rad/s
                jointStateMsg->effort.data[j] = isaacsim::core::includes::math::roundNearest(
                    jointEfforts[j] * stageUnits * stageUnits, 10000.0); // N*m
            }
        }
    }
}

size_t Ros2JointStateMessageImpl::getNumJoints()
{
    if (!m_msg)
    {
        return 0;
    }
    sensor_msgs__msg__JointState* jointStateMsg = static_cast<sensor_msgs__msg__JointState*>(m_msg);
    return jointStateMsg->name.size;
}

bool Ros2JointStateMessageImpl::checkValid()
{
    if (!m_msg)
    {
        return false;
    }
    sensor_msgs__msg__JointState* jointStateMsg = static_cast<sensor_msgs__msg__JointState*>(m_msg);
    const size_t numActuators = jointStateMsg->name.size;

    if (jointStateMsg->position.size != numActuators && jointStateMsg->velocity.size != numActuators &&
        jointStateMsg->effort.size != numActuators)
    {
        return false;
    }
    return true;
}

void Ros2JointStateMessageImpl::readData(std::vector<char*>& jointNames,
                                         double* jointPositions,
                                         double* jointVelocities,
                                         double* jointEfforts,
                                         double& timeStamp)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__JointState* jointStateMsg = static_cast<sensor_msgs__msg__JointState*>(m_msg);
    const size_t numActuators = jointStateMsg->name.size;

    if (numActuators == 0)
    {
        return;
    }

    jointNames.clear(); // Make sure vector is reset before filling in names
    for (size_t i = 0; i < numActuators; i++)
    {
        char* name = jointStateMsg->name.data[i].data;
        jointNames.push_back(name);
    }
    // Resize for the array was called before writeData in the subscriber callback
    if (jointStateMsg->position.size == numActuators)
    {
        std::memcpy(jointPositions, jointStateMsg->position.data, numActuators * sizeof(double));
    }
    else if (jointPositions)
    {
        // Set to some sentinel value to indicate no data
        for (size_t i = 0; i < numActuators; i++)
        {
            jointPositions[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    // Resize for the array was called before writeData in the subscriber callback
    if (jointStateMsg->velocity.size == numActuators)
    {
        std::memcpy(jointVelocities, jointStateMsg->velocity.data, numActuators * sizeof(double));
    }
    else if (jointVelocities)
    {
        for (size_t i = 0; i < numActuators; i++)
        {
            jointVelocities[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    // Resize for the array was called before writeData in the subscriber callback
    if (jointStateMsg->effort.size == numActuators)
    {
        std::memcpy(jointEfforts, jointStateMsg->effort.data, numActuators * sizeof(double));
    }
    else if (jointEfforts)
    {
        for (size_t i = 0; i < numActuators; i++)
        {
            jointEfforts[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    timeStamp = jointStateMsg->header.stamp.sec;
}

Ros2JointStateMessageImpl::~Ros2JointStateMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__JointState__destroy(static_cast<sensor_msgs__msg__JointState*>(m_msg));
}

// PointCloud2 message
Ros2PointCloudMessageImpl::Ros2PointCloudMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "PointCloud2")
{
    m_msg = sensor_msgs__msg__PointCloud2__create();
}

const void* Ros2PointCloudMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2);
}

void Ros2PointCloudMessageImpl::generateBuffer(const double& timeStamp,
                                               const std::string& frameId,
                                               const size_t& width,
                                               const size_t& height,
                                               const uint32_t& pointStep)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__PointCloud2* pointCloudMsg = static_cast<sensor_msgs__msg__PointCloud2*>(m_msg);

    pointCloudMsg->is_dense = true;
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), pointCloudMsg->header);
    pointCloudMsg->height = 1;
    pointCloudMsg->point_step = static_cast<uint32_t>(sizeof(pxr::GfVec3f));
    pointCloudMsg->width = static_cast<uint32_t>(width);

    pointCloudMsg->row_step = pointCloudMsg->point_step * pointCloudMsg->width;

    size_t totalBytes = width * sizeof(pxr::GfVec3f);
    pointCloudMsg->data.size = totalBytes;
    pointCloudMsg->data.capacity = totalBytes;
    m_buffer.resize(totalBytes);
    pointCloudMsg->data.data = &m_buffer[0];

    sensor_msgs__msg__PointField__Sequence__init(&pointCloudMsg->fields, 3);

    Ros2MessageInterfaceImpl::writeRosString("x", pointCloudMsg->fields.data[0].name);
    Ros2MessageInterfaceImpl::writeRosString("y", pointCloudMsg->fields.data[1].name);
    Ros2MessageInterfaceImpl::writeRosString("z", pointCloudMsg->fields.data[2].name);

    pointCloudMsg->fields.data[0].count = 1;
    pointCloudMsg->fields.data[1].count = 1;
    pointCloudMsg->fields.data[2].count = 1;

    pointCloudMsg->fields.data[0].datatype = sensor_msgs__msg__PointField__FLOAT32;
    pointCloudMsg->fields.data[1].datatype = sensor_msgs__msg__PointField__FLOAT32;
    pointCloudMsg->fields.data[2].datatype = sensor_msgs__msg__PointField__FLOAT32;


    pointCloudMsg->fields.data[0].offset = 0;
    pointCloudMsg->fields.data[1].offset = 4;
    pointCloudMsg->fields.data[2].offset = 8;
}

Ros2PointCloudMessageImpl::~Ros2PointCloudMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__PointCloud2* pointCloudMsg = static_cast<sensor_msgs__msg__PointCloud2*>(m_msg);
    // memory is managed by std::vector, clear this so destruction doesn't deallocate
    pointCloudMsg->data.size = 0;
    pointCloudMsg->data.capacity = 0;
    pointCloudMsg->data.data = nullptr;
    sensor_msgs__msg__PointCloud2__destroy(pointCloudMsg);
}

// LaserScan message
Ros2LaserScanMessageImpl::Ros2LaserScanMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "LaserScan")
{
    m_msg = sensor_msgs__msg__LaserScan__create();
}

const void* Ros2LaserScanMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);
}

void Ros2LaserScanMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__LaserScan* laserScanMsg = static_cast<sensor_msgs__msg__LaserScan*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), laserScanMsg->header);
}

void Ros2LaserScanMessageImpl::generateBuffers(const size_t buffSize)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__LaserScan* laserScanMsg = static_cast<sensor_msgs__msg__LaserScan*>(m_msg);

    laserScanMsg->ranges.size = buffSize;
    laserScanMsg->ranges.capacity = buffSize;
    m_rangeData.resize(buffSize);
    laserScanMsg->ranges.data = m_rangeData.data();

    laserScanMsg->intensities.size = buffSize;
    laserScanMsg->intensities.capacity = buffSize;
    m_intensitiesData.resize(buffSize);
    laserScanMsg->intensities.data = m_intensitiesData.data();
}

void Ros2LaserScanMessageImpl::writeData(const pxr::GfVec2f& azimuthRange,
                                         const float& rotationRate,
                                         const pxr::GfVec2f& depthRange,
                                         float horizontalResolution,
                                         float horizontalFov)
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__LaserScan* laserScanMsg = static_cast<sensor_msgs__msg__LaserScan*>(m_msg);
    float degToRadF = static_cast<float>(M_PI / 180.0f);

    laserScanMsg->angle_min = azimuthRange[0] * degToRadF;
    laserScanMsg->angle_max = azimuthRange[1] * degToRadF;

    laserScanMsg->scan_time = rotationRate ? 1.0f / rotationRate : 0.0f;
    laserScanMsg->range_min = depthRange[0];
    laserScanMsg->range_max = depthRange[1];

    laserScanMsg->angle_increment = horizontalResolution * degToRadF;
    laserScanMsg->time_increment = (horizontalFov / 360.0f * laserScanMsg->scan_time) / laserScanMsg->ranges.size;
}

Ros2LaserScanMessageImpl::~Ros2LaserScanMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    sensor_msgs__msg__LaserScan* laserScanMsg = static_cast<sensor_msgs__msg__LaserScan*>(m_msg);
    // Lifetime of memory is not managed by the message as we use std vectors
    laserScanMsg->ranges.size = 0;
    laserScanMsg->ranges.capacity = 0;
    laserScanMsg->ranges.data = nullptr;
    laserScanMsg->intensities.size = 0;
    laserScanMsg->intensities.capacity = 0;
    laserScanMsg->intensities.data = nullptr;
    sensor_msgs__msg__LaserScan__destroy(laserScanMsg);
}

// Full TFMessage message
// struct TfTransformStamped
// {
//     double timeStamp;
//     std::string parentFrame;
//     std::string childFrame;
//     geometry_msgs__msg__Transform transform;
// };

Ros2TfTreeMessageImpl::Ros2TfTreeMessageImpl() : Ros2MessageInterfaceImpl("tf2_msgs", "msg", "TFMessage")
{
    m_msg = tf2_msgs__msg__TFMessage__create();
}

const void* Ros2TfTreeMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage);
}

void Ros2TfTreeMessageImpl::writeData(const double& timeStamp, std::vector<TfTransformStamped>& transforms)
{
    if (!m_msg)
    {
        return;
    }
    tf2_msgs__msg__TFMessage* tfMsg = static_cast<tf2_msgs__msg__TFMessage*>(m_msg);
    geometry_msgs__msg__TransformStamped__Sequence__init(&tfMsg->transforms, transforms.size());

    for (size_t i = 0; i < transforms.size(); i++)
    {
        Ros2MessageInterfaceImpl::writeRosHeader(
            transforms[i].parentFrame, static_cast<int64_t>(timeStamp * 1e9), tfMsg->transforms.data[i].header);
        Ros2MessageInterfaceImpl::writeRosString(transforms[i].childFrame, tfMsg->transforms.data[i].child_frame_id);

        tfMsg->transforms.data[i].transform.translation.x = transforms[i].translationX;
        tfMsg->transforms.data[i].transform.translation.y = transforms[i].translationY;
        tfMsg->transforms.data[i].transform.translation.z = transforms[i].translationZ;

        tfMsg->transforms.data[i].transform.rotation.x = transforms[i].rotationX;
        tfMsg->transforms.data[i].transform.rotation.y = transforms[i].rotationY;
        tfMsg->transforms.data[i].transform.rotation.z = transforms[i].rotationZ;
        tfMsg->transforms.data[i].transform.rotation.w = transforms[i].rotationW;
    }
}

void Ros2TfTreeMessageImpl::readData(std::vector<TfTransformStamped>& transforms)
{
    if (!m_msg)
    {
        return;
    }
    tf2_msgs__msg__TFMessage* tfMsg = static_cast<tf2_msgs__msg__TFMessage*>(m_msg);
    const size_t numTransform = tfMsg->transforms.size;
    transforms.resize(numTransform);

    for (size_t i = 0; i < numTransform; i++)
    {
        transforms[i].parentFrame = std::string(tfMsg->transforms.data[i].header.frame_id.data);
        transforms[i].childFrame = std::string(tfMsg->transforms.data[i].child_frame_id.data);

        transforms[i].translationX = tfMsg->transforms.data[i].transform.translation.x;
        transforms[i].translationY = tfMsg->transforms.data[i].transform.translation.y;
        transforms[i].translationZ = tfMsg->transforms.data[i].transform.translation.z;

        transforms[i].rotationX = tfMsg->transforms.data[i].transform.rotation.x;
        transforms[i].rotationY = tfMsg->transforms.data[i].transform.rotation.y;
        transforms[i].rotationZ = tfMsg->transforms.data[i].transform.rotation.z;
        transforms[i].rotationW = tfMsg->transforms.data[i].transform.rotation.w;
    }
}

Ros2TfTreeMessageImpl::~Ros2TfTreeMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    tf2_msgs__msg__TFMessage__destroy(static_cast<tf2_msgs__msg__TFMessage*>(m_msg));
}

// Twist message
Ros2TwistMessageImpl::Ros2TwistMessageImpl() : Ros2MessageInterfaceImpl("geometry_msgs", "msg", "Twist")
{
    m_msg = geometry_msgs__msg__Twist__create();
}

const void* Ros2TwistMessageImpl::getTypeSupportHandle()
{
    return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
}

void Ros2TwistMessageImpl::readData(pxr::GfVec3d& linearVelocity, pxr::GfVec3d& angularVelocity)
{
    if (!m_msg)
    {
        return;
    }
    geometry_msgs__msg__Twist* twistMsg = static_cast<geometry_msgs__msg__Twist*>(m_msg);

    linearVelocity[0] = twistMsg->linear.x;
    linearVelocity[1] = twistMsg->linear.y;
    linearVelocity[2] = twistMsg->linear.z;

    angularVelocity[0] = twistMsg->angular.x;
    angularVelocity[1] = twistMsg->angular.y;
    angularVelocity[2] = twistMsg->angular.z;
}

Ros2TwistMessageImpl::~Ros2TwistMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    geometry_msgs__msg__Twist__destroy(static_cast<geometry_msgs__msg__Twist*>(m_msg));
}

// AckermannDriveStamped message
Ros2AckermannDriveStampedMessageImpl::Ros2AckermannDriveStampedMessageImpl()
    : Ros2MessageInterfaceImpl("ackermann_msgs", "msg", "AckermannDriveStamped")
{
    m_msg = create();
}

const void* Ros2AckermannDriveStampedMessageImpl::getTypeSupportHandle()
{
    return getTypeSupportHandleDynamic();
}

void Ros2AckermannDriveStampedMessageImpl::readData(double& timeStamp,
                                                    std::string& frameId,
                                                    double& steeringAngle,
                                                    double& steeringAngleVelocity,
                                                    double& speed,
                                                    double& acceleration,
                                                    double& jerk)
{
    if (!m_msg)
    {
        return;
    }
    ackermann_msgs__msg__AckermannDriveStamped* ackermannDriveMsg =
        static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg);

    frameId = ackermannDriveMsg->header.frame_id.data;
    timeStamp = ackermannDriveMsg->header.stamp.sec + ackermannDriveMsg->header.stamp.nanosec / 1e9;

    steeringAngle = ackermannDriveMsg->drive.steering_angle;
    steeringAngleVelocity = ackermannDriveMsg->drive.steering_angle_velocity;
    speed = ackermannDriveMsg->drive.speed;
    acceleration = ackermannDriveMsg->drive.acceleration;
    jerk = ackermannDriveMsg->drive.jerk;
}

void Ros2AckermannDriveStampedMessageImpl::writeHeader(const double timeStamp, const std::string& frameId)
{
    if (!m_msg)
    {
        return;
    }
    ackermann_msgs__msg__AckermannDriveStamped* ackermannDriveMsg =
        static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg);
    Ros2MessageInterfaceImpl::writeRosHeader(frameId, static_cast<int64_t>(timeStamp * 1e9), ackermannDriveMsg->header);
}

void Ros2AckermannDriveStampedMessageImpl::writeData(const double& steeringAngle,
                                                     const double& steeringAngleVelocity,
                                                     const double& speed,
                                                     const double& acceleration,
                                                     const double& jerk)
{
    if (!m_msg)
    {
        return;
    }
    ackermann_msgs__msg__AckermannDriveStamped* ackermannDriveMsg =
        static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg);

    ackermannDriveMsg->drive.steering_angle = static_cast<float>(steeringAngle);
    ackermannDriveMsg->drive.steering_angle_velocity = static_cast<float>(steeringAngleVelocity);
    ackermannDriveMsg->drive.speed = static_cast<float>(speed);
    ackermannDriveMsg->drive.acceleration = static_cast<float>(acceleration);
    ackermannDriveMsg->drive.jerk = static_cast<float>(jerk);
}

Ros2AckermannDriveStampedMessageImpl::~Ros2AckermannDriveStampedMessageImpl()
{
    if (!m_msg)
    {
        return;
    }
    destroy(static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg));
}

} // namespace bridge
} // namespace ros2
} // namespace isaacsim
