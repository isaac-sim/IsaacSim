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

#include <rcl/rcl.h>
#include <sensor_msgs/msg/camera_info.h>

#include <cstdio>
#include <cstring>
#include <limits>

namespace isaacsim
{
namespace ros2
{
namespace bridge
{


// Generic macro to define ensureSeqSize for a concrete sequence type
#define ISAACSIM_DEFINE_ENSURE_SEQ_SIZE(SeqType, InitFn, FiniFn)                                                       \
    inline bool ensureSeqSize(SeqType& sequence, size_t requiredSize)                                                  \
    {                                                                                                                  \
        if (sequence.data == NULL)                                                                                     \
        {                                                                                                              \
            return InitFn(&sequence, requiredSize);                                                                    \
        }                                                                                                              \
        if (sequence.capacity < requiredSize)                                                                          \
        {                                                                                                              \
            FiniFn(&sequence);                                                                                         \
            return InitFn(&sequence, requiredSize);                                                                    \
        }                                                                                                              \
        sequence.size = requiredSize;                                                                                  \
        return true;                                                                                                   \
    }

// Doubles
ISAACSIM_DEFINE_ENSURE_SEQ_SIZE(rosidl_runtime_c__double__Sequence,
                                rosidl_runtime_c__double__Sequence__init,
                                rosidl_runtime_c__double__Sequence__fini)

// Strings
ISAACSIM_DEFINE_ENSURE_SEQ_SIZE(rosidl_runtime_c__String__Sequence,
                                rosidl_runtime_c__String__Sequence__init,
                                rosidl_runtime_c__String__Sequence__fini)

// geometry_msgs/TransformStamped[]
ISAACSIM_DEFINE_ENSURE_SEQ_SIZE(geometry_msgs__msg__TransformStamped__Sequence,
                                geometry_msgs__msg__TransformStamped__Sequence__init,
                                geometry_msgs__msg__TransformStamped__Sequence__fini)

// sensor_msgs/PointField[]
ISAACSIM_DEFINE_ENSURE_SEQ_SIZE(sensor_msgs__msg__PointField__Sequence,
                                sensor_msgs__msg__PointField__Sequence__init,
                                sensor_msgs__msg__PointField__Sequence__fini)

#undef ISAACSIM_DEFINE_ENSURE_SEQ_SIZE

// Dynamic symbol-based ensure function for sequences whose init/fini must be invoked via a
// dynamically loaded library (e.g., vision_msgs sequences).
template <typename LibraryLike, typename SequenceT>
inline bool ensureSeqSizeDynamic(
    LibraryLike& library, const char* initSymbol, const char* finiSymbol, SequenceT& sequence, size_t requiredSize)
{
    if (sequence.data == NULL)
    {
        return library.template callSymbolWithArg<bool>(initSymbol, &sequence, requiredSize);
    }
    if (sequence.capacity < requiredSize)
    {
        library.template callSymbolWithArg<void>(finiSymbol, &sequence);
        return library.template callSymbolWithArg<bool>(initSymbol, &sequence, requiredSize);
    }
    sequence.size = requiredSize;
    return true;
}

// Convenience macro to improve call-site readability
#define ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC(lib, seq, n, initSym, finiSym)                                                \
    ensureSeqSizeDynamic((lib), (initSym), (finiSym), (seq), (n))

// Ensure for parent sequences that contain an inner sequence that must be finalized before
// re-initializing the parent (when growing capacity). The accessor returns a reference to the
// inner sequence within a parent element at index i.
template <typename LibraryLike, typename ParentSeq, typename GetInnerSequenceFn>
inline bool ensureSeqSizeDynamicWithInnerFini(LibraryLike& library,
                                              const char* parentInitSymbol,
                                              const char* parentFiniSymbol,
                                              ParentSeq& parentSequence,
                                              size_t requiredSize,
                                              GetInnerSequenceFn getInnerSequence,
                                              const char* innerFiniSymbol)
{
    if (parentSequence.data == NULL)
    {
        return library.template callSymbolWithArg<bool>(parentInitSymbol, &parentSequence, requiredSize);
    }
    if (parentSequence.capacity < requiredSize)
    {
        // Finalize inner sequences for existing elements to avoid leaks before finalizing the parent
        for (size_t i = 0; i < parentSequence.size; ++i)
        {
            auto& inner = getInnerSequence(parentSequence.data[i]);
            library.template callSymbolWithArg<void>(innerFiniSymbol, &inner);
        }
        library.template callSymbolWithArg<void>(parentFiniSymbol, &parentSequence);
        return library.template callSymbolWithArg<bool>(parentInitSymbol, &parentSequence, requiredSize);
    }
    parentSequence.size = requiredSize;
    return true;
}

#define ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC_WITH_INNER_FINI(                                                              \
    lib, seq, n, parentInitSym, parentFiniSym, getInnerFn, innerFiniSym)                                               \
    ensureSeqSizeDynamicWithInnerFini((lib), (parentInitSym), (parentFiniSym), (seq), (n), (getInnerFn), (innerFiniSym))


// Clock message
Ros2ClockMessageImpl::Ros2ClockMessageImpl() : Ros2MessageInterfaceImpl("rosgraph_msgs", "msg", "Clock")
{
    m_msg = rosgraph_msgs__msg__Clock__create();
}

Ros2ClockMessageImpl::~Ros2ClockMessageImpl()
{
    if (m_msg)
    {
        rosgraph_msgs__msg__Clock__destroy(static_cast<rosgraph_msgs__msg__Clock*>(m_msg));
    }
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
        timeStamp = 0.0;
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

void Ros2ImuMessageImpl::writeAcceleration(bool covariance, const std::vector<double>& acceleration)
{
    if (!m_msg)
    {
        return;
    }
    if (!covariance && acceleration.size() < 3)
    {
        fprintf(stderr, "[Ros2ImuMessage] Expected 3 values for acceleration, got %zu\n", acceleration.size());
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

void Ros2ImuMessageImpl::writeVelocity(bool covariance, const std::vector<double>& velocity)
{
    if (!m_msg)
    {
        return;
    }
    if (!covariance && velocity.size() < 3)
    {
        fprintf(stderr, "[Ros2ImuMessage] Expected 3 values for velocity, got %zu\n", velocity.size());
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

void Ros2ImuMessageImpl::writeOrientation(bool covariance, const std::vector<double>& orientation)
{
    if (!m_msg)
    {
        return;
    }
    if (!covariance && orientation.size() < 4)
    {
        fprintf(stderr, "[Ros2ImuMessage] Expected 4 values for orientation, got %zu\n", orientation.size());
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
    if (m_msg)
    {
        sensor_msgs__msg__Imu__destroy(static_cast<sensor_msgs__msg__Imu*>(m_msg));
    }
}

// CameraInfo message
Ros2CameraInfoMessageImpl::Ros2CameraInfoMessageImpl() : Ros2MessageInterfaceImpl("sensor_msgs", "msg", "CameraInfo")
{
    m_msg = sensor_msgs__msg__CameraInfo__create();
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
        fprintf(stderr, "[Ros2CameraInfoMessage] writeIntrinsicMatrix: input array is null\n");
        return;
    }

    if (arraySize != 9)
    {
        fprintf(stderr, "[Ros2CameraInfoMessage] Invalid array size %zu, expected 9 for 3x3 matrix\n", arraySize);
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
    else
    {
        // Clear the sequence safely
        cameraInfoMsg->d.data = nullptr;
        cameraInfoMsg->d.size = 0;
        cameraInfoMsg->d.capacity = 0;
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
        fprintf(stderr, "[Ros2CameraInfoMessage] writeProjectionMatrix: input array is null\n");
        return;
    }

    if (arraySize != 12)
    {
        fprintf(stderr,
                "[Ros2CameraInfoMessage] writeProjectionMatrix: invalid array size %zu, expected 12 for 3x4 matrix\n",
                arraySize);
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
        fprintf(stderr, "[Ros2CameraInfoMessage] writeRectificationMatrix: input array is null\n");
        return;
    }

    if (arraySize != 9)
    {
        fprintf(stderr,
                "[Ros2CameraInfoMessage] writeRectificationMatrix: invalid array size %zu, expected 9 for 3x3 matrix\n",
                arraySize);
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
    imageMsg->data.data = m_totalBytes > 0 ? m_buffer.data() : nullptr;
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
        fprintf(stderr, "Image encoding error: %s\n", e.what());
        return;
    }
    int byteDepth = bitDepth / 8;

    uint32_t step = width * channels * byteDepth;
    imageMsg->step = step;
    m_totalBytes = step * height;
#endif
}

void Ros2NitrosBridgeImageMessageImpl::writeData(const std::vector<int32_t>& imageData)
{
    if (!m_msg || imageData.empty())
    {
        return;
    }
#if !defined(_WIN32)
    isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage* imageMsg =
        static_cast<isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeImage*>(m_msg);

    m_imageData.resize(imageData.size());
    std::memcpy(m_imageData.data(), imageData.data(), imageData.size() * sizeof(int32_t));

    imageMsg->data.size = m_imageData.size();
    imageMsg->data.capacity = m_imageData.size();
    imageMsg->data.data = m_imageData.data();
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
    float occlusionRatio; // Unused but needed to match the size of the incoming void* struct
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
    if (bboxArray == nullptr && numBoxes > 0)
    {
        fprintf(stderr, "[Ros2BoundingBox2DMessage] bboxArray is null for numBoxes=%zu\n", numBoxes);
        return;
    }
    vision_msgs__msg__Detection2DArray* detectionMsg = static_cast<vision_msgs__msg__Detection2DArray*>(m_msg);

    // Ensure detection sequence capacity and reuse where possible. If parent capacity must grow,
    // finalize inner ObjectHypothesisWithPose sequences first to avoid leaks.
    if (!ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC_WITH_INNER_FINI(
            *m_generatorLibrary, detectionMsg->detections, numBoxes, "vision_msgs__msg__Detection2D__Sequence__init",
            "vision_msgs__msg__Detection2D__Sequence__fini",
            [](vision_msgs__msg__Detection2D& det) -> vision_msgs__msg__ObjectHypothesisWithPose__Sequence&
            { return det.results; },
            "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__fini"))
    {
        fprintf(stderr, "[Ros2BoundingBox2DMessage] Failed to ensure detection sequence\n");
        return;
    }

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

        // Ensure results sequence has capacity 1 and reuse if possible
        if (!ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC(*m_generatorLibrary, detectionMsg->detections.data[i].results, 1,
                                              "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__init",
                                              "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__fini"))
        {
            fprintf(stderr, "[Ros2BoundingBox2DMessage] Warning: Failed to ensure results sequence for box %zu\n", i);
            continue;
        }

        detectionMsg->detections.data[i].results.data[0].hypothesis.score = 1.0;

        // Assign semantic ID
        Ros2MessageInterfaceImpl::writeRosString(
            std::to_string(box.semanticId), detectionMsg->detections.data[i].results.data[0].hypothesis.class_id);
    }
}

Ros2BoundingBox2DMessageImpl::~Ros2BoundingBox2DMessageImpl()
{
    if (m_msg)
    {
        destroy(static_cast<vision_msgs__msg__Detection2DArray*>(m_msg));
    }
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
    float occlusionRatio; // Unused but needed to match the size of the incoming void* struct
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
    if (bboxArray == nullptr && numBoxes > 0)
    {
        fprintf(stderr, "[Ros2BoundingBox3DMessage] bboxArray is null for numBoxes=%zu\n", numBoxes);
        return;
    }
    vision_msgs__msg__Detection3DArray* detectionMsg = static_cast<vision_msgs__msg__Detection3DArray*>(m_msg);

    // Ensure detection sequence capacity and reuse where possible. If parent capacity must grow,
    // finalize inner ObjectHypothesisWithPose sequences first to avoid leaks.
    if (!ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC_WITH_INNER_FINI(
            *m_generatorLibrary, detectionMsg->detections, numBoxes, "vision_msgs__msg__Detection3D__Sequence__init",
            "vision_msgs__msg__Detection3D__Sequence__fini",
            [](vision_msgs__msg__Detection3D& det) -> vision_msgs__msg__ObjectHypothesisWithPose__Sequence&
            { return det.results; },
            "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__fini"))
    {
        fprintf(stderr, "[Ros2BoundingBox3DMessage] Failed to ensure detection sequence\n");
        return;
    }
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

        // Ensure results sequence has capacity 1 and reuse if possible
        if (!ISAACSIM_ENSURE_SEQ_SIZE_DYNAMIC(*m_generatorLibrary, detectionMsg->detections.data[i].results, 1,
                                              "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__init",
                                              "vision_msgs__msg__ObjectHypothesisWithPose__Sequence__fini"))
        {
            fprintf(stderr, "[Ros2BoundingBox3DMessage] Warning: Failed to ensure results sequence for box %zu\n", i);
            continue;
        }

        detectionMsg->detections.data[i].results.data[0].hypothesis.score = 1.0;

        // Assign semantic ID
        Ros2MessageInterfaceImpl::writeRosString(
            std::to_string(box.semanticId), detectionMsg->detections.data[i].results.data[0].hypothesis.class_id);
    }
}

Ros2BoundingBox3DMessageImpl::~Ros2BoundingBox3DMessageImpl()
{
    if (m_msg)
    {
        destroy(static_cast<vision_msgs__msg__Detection3DArray*>(m_msg));
    }
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

    // Assign child frame ID
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
    if (m_msg)
    {
        nav_msgs__msg__Odometry__destroy(static_cast<nav_msgs__msg__Odometry*>(m_msg));
    }
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

    // Ensure capacity for a single transform and reuse if possible
    if (!ensureSeqSize(tfMsg->transforms, 1))
    {
        fprintf(stderr, "[Ros2RawTfTreeMessage] Failed to ensure transform sequence\n");
        return;
    }

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
    if (m_msg)
    {
        tf2_msgs__msg__TFMessage__destroy(static_cast<tf2_msgs__msg__TFMessage*>(m_msg));
    }
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

    // Assign string data
    Ros2MessageInterfaceImpl::writeRosString(data, stringMsg->data);
}

Ros2SemanticLabelMessageImpl::~Ros2SemanticLabelMessageImpl()
{
    if (m_msg)
    {
        std_msgs__msg__String__destroy(static_cast<std_msgs__msg__String*>(m_msg));
    }
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
                             omni::physics::tensors::TensorDataType type,
                             int cudaDeviceIndex)
{
    buffer.resize(numElements);
    tensorDesc.dtype = type;
    tensorDesc.numDims = 1;
    tensorDesc.dims[0] = numElements;
    tensorDesc.data = buffer.data();
    tensorDesc.ownData = true;
    tensorDesc.device = cudaDeviceIndex;
}

void Ros2JointStateMessageImpl::writeData(const double& timeStamp,
                                          omni::physics::tensors::IArticulationView* articulation,
                                          pxr::UsdStageWeakPtr stage,
                                          std::vector<float>& jointPositions,
                                          std::vector<float>& jointVelocities,
                                          std::vector<float>& jointEfforts,
                                          std::vector<uint8_t>& dofTypes,
                                          const double& stageUnits,
                                          const int& cudaDeviceIndex)
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
    createTensorDesc(positionTensor, jointPositions, numDofs, omni::physics::tensors::TensorDataType::eFloat32, cudaDeviceIndex);
    createTensorDesc(velocityTensor, jointVelocities, numDofs, omni::physics::tensors::TensorDataType::eFloat32, cudaDeviceIndex);
    createTensorDesc(effortTensor, jointEfforts, numDofs, omni::physics::tensors::TensorDataType::eFloat32, cudaDeviceIndex);
    // The DOF type data always lives on host.
    createTensorDesc(dofTypeTensor, dofTypes, numDofs, omni::physics::tensors::TensorDataType::eUint8, -1);
    bool hasDofStates = true;
    if (!articulation->getDofPositions(&positionTensor))
    {
        fprintf(stderr, "[Ros2JointStateMessage] Failed to get dof positions\n");
        hasDofStates = false;
    }
    if (!articulation->getDofVelocities(&velocityTensor))
    {
        fprintf(stderr, "[Ros2JointStateMessage] Failed to get dof velocities\n");
        hasDofStates = false;
    }
    if (!articulation->getDofProjectedJointForces(&effortTensor))
    {
        fprintf(stderr, "[Ros2JointStateMessage] Failed to get dof efforts\n");
        hasDofStates = false;
    }
    if (!articulation->getDofTypes(&dofTypeTensor))
    {
        fprintf(stderr, "[Ros2JointStateMessage] Failed to get dof types\n");
        hasDofStates = false;
    }

    // Ensure sequences have enough capacity and reuse allocations where possible
    if (!ensureSeqSize(jointStateMsg->name, numDofs) || !ensureSeqSize(jointStateMsg->position, numDofs) ||
        !ensureSeqSize(jointStateMsg->velocity, numDofs) || !ensureSeqSize(jointStateMsg->effort, numDofs))
    {
        fprintf(stderr, "[Ros2JointStateMessage] Failed to ensure sequence capacities\n");
        return;
    }

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
    timeStamp = jointStateMsg->header.stamp.sec + jointStateMsg->header.stamp.nanosec / 1e9;
}

Ros2JointStateMessageImpl::~Ros2JointStateMessageImpl()
{
    if (m_msg)
    {
        sensor_msgs__msg__JointState__destroy(static_cast<sensor_msgs__msg__JointState*>(m_msg));
    }
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

    m_totalBytes = width * sizeof(pxr::GfVec3f);

    // Use buffer-backed sequence for safe memory management
    m_buffer.resize(m_totalBytes);
    pointCloudMsg->data.size = m_totalBytes;
    pointCloudMsg->data.capacity = m_totalBytes;
    pointCloudMsg->data.data = m_buffer.data();

    // Ensure fields sequence has capacity 3 and reuse if possible
    if (!ensureSeqSize(pointCloudMsg->fields, 3))
    {
        fprintf(stderr, "[Ros2PointCloudMessage] Failed to ensure fields sequence\n");
        return;
    }

    // Assign field names
    const char* fieldNames[] = { "x", "y", "z" };
    for (int i = 0; i < 3; i++)
    {
        Ros2MessageInterfaceImpl::writeRosString(fieldNames[i], pointCloudMsg->fields.data[i].name);
        pointCloudMsg->fields.data[i].count = 1;
        pointCloudMsg->fields.data[i].datatype = sensor_msgs__msg__PointField__FLOAT32;
        pointCloudMsg->fields.data[i].offset = i * 4;
    }
}

Ros2PointCloudMessageImpl::~Ros2PointCloudMessageImpl()
{
    if (m_msg)
    {
        sensor_msgs__msg__PointCloud2* pointCloudMsg = static_cast<sensor_msgs__msg__PointCloud2*>(m_msg);
        // memory is managed by std::vector, clear this so destruction doesn't deallocate
        pointCloudMsg->data.size = 0;
        pointCloudMsg->data.capacity = 0;
        pointCloudMsg->data.data = nullptr;
        sensor_msgs__msg__PointCloud2__destroy(pointCloudMsg);
    }
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


    m_rangeData.resize(buffSize);
    laserScanMsg->ranges.size = buffSize;
    laserScanMsg->ranges.capacity = buffSize;
    laserScanMsg->ranges.data = m_rangeData.data();

    m_intensitiesData.resize(buffSize);
    laserScanMsg->intensities.size = buffSize;
    laserScanMsg->intensities.capacity = buffSize;
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
    if (m_msg)
    {
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

    // Ensure capacity for transforms and reuse if possible
    if (!ensureSeqSize(tfMsg->transforms, transforms.size()))
    {
        fprintf(stderr, "[Ros2TfTreeMessage] Failed to ensure transform sequence\n");
        return;
    }

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
    if (m_msg)
    {
        tf2_msgs__msg__TFMessage__destroy(static_cast<tf2_msgs__msg__TFMessage*>(m_msg));
    }
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
        linearVelocity = pxr::GfVec3d(0.0);
        angularVelocity = pxr::GfVec3d(0.0);
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
    if (m_msg)
    {
        geometry_msgs__msg__Twist__destroy(static_cast<geometry_msgs__msg__Twist*>(m_msg));
    }
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
        // Set default values
        timeStamp = 0.0;
        frameId.clear();
        steeringAngle = 0.0;
        steeringAngleVelocity = 0.0;
        speed = 0.0;
        acceleration = 0.0;
        jerk = 0.0;
        return;
    }
    ackermann_msgs__msg__AckermannDriveStamped* ackermannDriveMsg =
        static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg);

    frameId = ackermannDriveMsg->header.frame_id.data ? ackermannDriveMsg->header.frame_id.data : "";
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
    if (m_msg)
    {
        destroy(static_cast<ackermann_msgs__msg__AckermannDriveStamped*>(m_msg));
    }
}

} // namespace bridge
} // namespace ros2
} // namespace isaacsim
