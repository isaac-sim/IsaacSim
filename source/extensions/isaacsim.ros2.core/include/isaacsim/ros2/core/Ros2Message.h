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

#include <isaacsim/core/includes/Buffer.h>
#include <isaacsim/ros2/core/Ros2Types.h>

namespace omni::physics::tensors
{
class IArticulationView;
}

namespace isaacsim
{
namespace ros2
{
namespace core
{
/**
 * @class Ros2ClockMessage
 * @brief Class implementing a `rosgraph_msgs/msg/Clock` message.
 * @details
 * Provides functionality to read and write ROS 2 Clock messages that contain
 * simulation time information.
 */
class Ros2ClockMessage : public Ros2Message
{
public:
    /**
     * @brief Read the message field values.
     * @details
     * Extracts the timestamp value from a ROS 2 Clock message.
     *
     * @param[out] timeStamp Time (seconds).
     */
    virtual void readData(double& timeStamp) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the timestamp value in a ROS 2 Clock message.
     *
     * @param[in] timeStamp Time (seconds).
     */
    virtual void writeData(double timeStamp) = 0;
};

/**
 * @class Ros2ImuMessage
 * @brief Class implementing a `sensor_msgs/msg/Imu` message.
 * @details
 * Provides functionality to write ROS 2 IMU messages that contain
 * orientation, angular velocity, and linear acceleration data.
 */
class Ros2ImuMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 IMU message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(double timeStamp, std::string& frameId) = 0;

    /**
     * @brief Write the `linear_acceleration` or its covariance.
     * @details
     * Sets the linear acceleration values or marks the covariance as unknown
     * in a ROS 2 IMU message.
     *
     * @param[in] covariance If true, only the element 0 of the associated covariance matrix will be written to -1, not
     * the acceleration (regardless of its value). If false, the acceleration values will be written.
     * @param[in] acceleration Linear acceleration.
     */
    virtual void writeAcceleration(bool covariance = false,
                                   const std::vector<double>& acceleration = std::vector<double>()) = 0;

    /**
     * @brief Write the `angular_velocity` or its covariance.
     * @details
     * Sets the angular velocity values or marks the covariance as unknown
     * in a ROS 2 IMU message.
     *
     * @param[in] covariance If true, only the element 0 of the associated covariance matrix will be written to -1, not
     * the velocity (regardless of its value). If false, the velocity values will be written.
     * @param[in] velocity Angular velocity.
     */
    virtual void writeVelocity(bool covariance = false, const std::vector<double>& velocity = std::vector<double>()) = 0;

    /**
     * @brief Write the `orientation` or its covariance.
     * @details
     * Sets the orientation values or marks the covariance as unknown
     * in a ROS 2 IMU message.
     *
     * @param[in] covariance If true, only the element 0 of the associated covariance matrix will be written to -1, not
     * the orientation (regardless of its value). If false, the orientation values will be written.
     * @param[in] orientation Orientation.
     */
    virtual void writeOrientation(bool covariance = false,
                                  const std::vector<double>& orientation = std::vector<double>()) = 0;
};

/**
 * @class Ros2CameraInfoMessage
 * @brief Class implementing a `sensor_msgs/msg/CameraInfo` message.
 * @details
 * Provides functionality to write ROS 2 CameraInfo messages that contain
 * camera calibration information such as resolution, intrinsic parameters,
 * distortion parameters, and projection matrices.
 */
class Ros2CameraInfoMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 CameraInfo message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Write the image dimensions (resolution).
     * @details
     * Sets the height and width fields in a ROS 2 CameraInfo message.
     *
     * @param[in] height Image height.
     * @param[in] width Image width.
     */
    virtual void writeResolution(const uint32_t height, const uint32_t width) = 0;

    /**
     * @brief Write the intrinsic camera matrix (`K`).
     * @details
     * Sets the 3x3 intrinsic camera matrix in a ROS 2 CameraInfo message.
     *
     * @param[in] array Flattened intrinsic camera matrix.
     * @param[in] arraySize Array size.
     */
    virtual void writeIntrinsicMatrix(const double array[], const size_t arraySize) = 0;

    /**
     * @brief Write the projection/camera matrix (`P`).
     * @details
     * Sets the 3x4 projection matrix in a ROS 2 CameraInfo message.
     *
     * @param[in] array Flattened projection/camera matrix.
     * @param[in] arraySize Array size.
     */
    virtual void writeProjectionMatrix(const double array[], const size_t arraySize) = 0;

    /**
     * @brief Write the rectification matrix (`R`).
     * @details
     * Sets the 3x3 rectification matrix in a ROS 2 CameraInfo message.
     *
     * @param[in] array Flattened rectification matrix.
     * @param[in] arraySize Array size.
     */
    virtual void writeRectificationMatrix(const double array[], const size_t arraySize) = 0;

    /**
     * @brief Write the distortion parameters (`D`).
     * @details
     * Sets the distortion parameters in a ROS 2 CameraInfo message based on
     * the specified distortion model.
     *
     * @param[in] array Distortion parameters (size depending on the distortion model).
     * @param[in] distortionModel Distortion model.
     */
    virtual void writeDistortionParameters(std::vector<double>& array, const std::string& distortionModel) = 0;

protected:
    /**
     * @brief Distortion Buffer (matrix data).
     */
    std::vector<double> m_distortionBuffer;
};

/**
 * @class Ros2ImageMessage
 * @brief Class implementing a `sensor_msgs/msg/Image` message.
 * @details
 * Provides functionality to write ROS 2 Image messages that contain
 * image data with specified encoding and dimensions.
 */
class Ros2ImageMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 Image message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Generate the buffer with optional CUDA pinned memory allocation.
     * @details
     * Allocates memory for the `data` field, and computes and fills in the values of the other message fields.
     * When usePinnedMemory is true, allocates CUDA (page-locked) memory for faster GPU transfers.
     *
     * @param[in] height Image height.
     * @param[in] width Image width.
     * @param[in] encoding Encoding of pixels.
     * @param[in] usePinnedMemory If true, allocate CUDA pinned memory instead of regular host memory.
     */
    virtual void generateBuffer(const uint32_t height,
                                const uint32_t width,
                                const std::string& encoding,
                                bool usePinnedMemory = false) = 0;

    /**
     * @brief Get the pointer to the buffer (matrix data).
     * @details
     * Provides direct access to the underlying buffer containing the image data.
     * Returns pinned memory pointer if allocated, otherwise returns vector buffer pointer.
     *
     * @return Pointer to the buffer.
     */
    void* getBufferPtr()
    {
        return m_isPinnedMemory ? m_pinnedBuffer : &m_buffer[0];
    }

    /**
     * @brief Get the total size (`step * height`) of the buffer, in bytes.
     * @details
     * Returns the total size of the image buffer in bytes.
     *
     * @return Buffer size.
     */
    size_t getTotalBytes()
    {
        return m_totalBytes;
    }

protected:
    /**
     * @brief Buffer (matrix data).
     */
    std::vector<uint8_t> m_buffer;

    /**
     * @brief Pinned memory buffer pointer.
     */
    void* m_pinnedBuffer = nullptr;

    /**
     * @brief Buffer size.
     */
    size_t m_totalBytes = 0;

    /**
     * @brief Flag indicating if buffer is using CUDA pinned memory.
     */
    bool m_isPinnedMemory = false;
};

/**
 * @class Ros2CompressedImageMessage
 * @brief Class implementing a `sensor_msgs/msg/CompressedImage` message.
 * @details
 * Provides functionality to write ROS 2 CompressedImage messages that contain
 * compressed image data (e.g., H264, HEVC encoded bitstreams).
 */
class Ros2CompressedImageMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 CompressedImage message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Write the compressed image data.
     * @details
     * Sets the format and data fields in a ROS 2 CompressedImage message.
     *
     * @param[in] data Pointer to the compressed image data (e.g., H264 bitstream).
     * @param[in] dataSize Size of the data in bytes.
     * @param[in] format Compression format string (e.g., "h264", "hevc", "jpeg").
     */
    virtual void writeData(const uint8_t* data, size_t dataSize, const std::string& format) = 0;
};

/**
 * @class Ros2NitrosBridgeImageMessage
 * @brief Class implementing a `isaac_ros_nitros_bridge_interfaces/msg/NitrosBridgeImage` message.
 * @details
 * Provides functionality to write ROS 2 NitrosBridgeImage messages that enable
 * zero-copy transfer of image data between processes using CUDA memory.
 */
class Ros2NitrosBridgeImageMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 NitrosBridgeImage message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Compute and fill in the values of the other message fields.
     * @details
     * This method is named the same as \ref Ros2ImageMessage::generateBuffer for compatibility.
     * Since the NitrosBridgeImage message does not define a buffer, no memory allocation is performed.
     * It only computes and fills in the values of the other message fields.
     *
     * @param[in] height Image height.
     * @param[in] width Image width.
     * @param[in] encoding Encoding of pixels.
     */
    virtual void generateBuffer(const uint32_t height, const uint32_t width, const std::string& encoding) = 0;

    /**
     * @brief Write the `data` field.
     * @details
     * Sets the data field containing the process ID and CUDA memory block file-descriptor
     * in a ROS 2 NitrosBridgeImage message.
     *
     * @param[in] data Calling process ID and the CUDA memory block file-descriptor.
     */
    virtual void writeData(const std::vector<int32_t>& data) = 0;

    /**
     * @brief Get the pointer to the buffer.
     * @details
     * This method is named the same as \ref Ros2ImageMessage::getBufferPtr for compatibility.
     * Since the NitrosBridgeImage message does not define a buffer, it always return `nullptr`.
     *
     * @return `nullptr`.
     */
    void* getBufferPtr()
    {
        return nullptr;
    }

    /**
     * @brief Get the total size (`step * height`) of the buffer, in bytes.
     * @details
     * Returns the total size of the image data in bytes.
     *
     * @return Buffer size.
     */
    size_t getTotalBytes()
    {
        return m_totalBytes;
    }

protected:
    /**
     * @brief Buffer size.
     */
    size_t m_totalBytes = 0;

    /**
     * @brief Calling process ID and the CUDA memory block file-descriptor.
     */
    std::vector<int32_t> m_imageData;
};

/**
 * @class Ros2BoundingBox2DMessage
 * @brief Class implementing a `vision_msgs/msg/Detection2DArray` message.
 * @details
 * Provides functionality to write ROS 2 Detection2DArray messages that contain
 * 2D bounding box detection information.
 */
class Ros2BoundingBox2DMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 Detection2DArray message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the detection array in a ROS 2 Detection2DArray message.
     *
     * @param[in] bboxArray Array of `Bbox2DData` struct.
     * @param[in] numBoxes Number of boxed defined in the array.
     */
    virtual void writeBboxData(const void* bboxArray, size_t numBoxes) = 0;
};

/**
 * @class Ros2BoundingBox3DMessage
 * @brief Class implementing a `vision_msgs/msg/Detection3DArray` message.
 * @details
 * Provides functionality to write ROS 2 Detection3DArray messages that contain
 * 3D bounding box detection information.
 */
class Ros2BoundingBox3DMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 Detection3DArray message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the detection array in a ROS 2 Detection3DArray message.
     *
     * @param[in] bboxArray Array of `Bbox3DData` struct.
     * @param[in] numBoxes Number of boxed defined in the array.
     */
    virtual void writeBboxData(const void* bboxArray, size_t numBoxes) = 0;
};

/**
 * @class Ros2JointStateMessage
 * @brief Class implementing a `sensor_msgs/msg/JointState` message.
 * @details
 * Provides functionality to read and write ROS 2 JointState messages that contain
 * joint positions, velocities, and efforts for articulated mechanisms.
 */
class Ros2JointStateMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the joint names, positions, velocities, efforts, and timestamp
     * in a ROS 2 JointState message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] articulation Articulation handler.
     * @param[in] stage USD stage.
     * @param[in] jointPositions Joint positions.
     * @param[in] jointVelocities Joint velocities.
     * @param[in] jointEfforts Joint efforts.
     * @param[in] dofTypes Articulation DOF types.
     * @param[in] stageUnits Unit scale of the stage.
     */
    virtual void writeData(const double& timeStamp,
                           omni::physics::tensors::IArticulationView* articulation,
                           pxr::UsdStageWeakPtr stage,
                           std::vector<float>& jointPositions,
                           std::vector<float>& jointVelocities,
                           std::vector<float>& jointEfforts,
                           std::vector<uint8_t>& dofTypes,
                           const double& stageUnits) = 0;

    /**
     * @brief Write the message from joint state arrays (e.g. from Isaac Read Joint State node).
     * @details
     * Sets the joint names, positions, velocities, efforts, and timestamp from the given arrays.
     * Does not use articulation or stage. Use when publishing from OmniGraph inputs.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] jointNames Joint name strings (size n).
     * @param[in] jointPositions Joint positions (rad or m), size n.
     * @param[in] jointVelocities Joint velocities (rad/s or m/s), size n.
     * @param[in] jointEfforts Joint efforts (Nm or N), size n.
     * @param[in] dofTypes Per-DOF type: 0 = revolute, 1 = prismatic; size n.
     * @param[in] stageMetersPerUnit Stage meters per USD unit (e.g. 0.01 for cm).
     */
    virtual void writeData(const double& timeStamp,
                           const std::vector<std::string>& jointNames,
                           const std::vector<double>& jointPositions,
                           const std::vector<double>& jointVelocities,
                           const std::vector<double>& jointEfforts,
                           const std::vector<uint8_t>& dofTypes,
                           double stageMetersPerUnit) = 0;

    /**
     * @brief Read the message field values.
     * @details
     * Extracts joint names, positions, velocities, efforts, and timestamp
     * from a ROS 2 JointState message.
     *
     * The size of the arrays to store the joint positions, velocities, and efforts must be equal to the number of
     * joints (see \ref Ros2JointStateMessage::getNumJoints) of the message before calling this method.
     *
     * @param[out] jointNames Joint names.
     * @param[out] jointPositions Joint positions.
     * @param[out] jointVelocities Joint velocities.
     * @param[out] jointEfforts Joint efforts.
     * @param[out] timeStamp Time (seconds).
     */
    virtual void readData(std::vector<char*>& jointNames,
                          double* jointPositions,
                          double* jointVelocities,
                          double* jointEfforts,
                          double& timeStamp) = 0;

    /**
     * @brief Get the number of joints in the message.
     * @details
     * Returns the number of joints in the message, derived from the size of the `name` field array.
     *
     * @return Number of joints.
     */
    virtual size_t getNumJoints() = 0;

    /**
     * @brief Check that the message field arrays have the same size.
     * @details
     * Verifies that the joint names, positions, velocities, and efforts arrays
     * all have the same size, indicating a valid message.
     *
     * @return Whether the message field arrays have the same size.
     */
    virtual bool checkValid() = 0;
};

/**
 * @class Ros2LaserScanMessage
 * @brief Class implementing a `sensor_msgs/msg/LaserScan` message.
 * @details
 * Provides functionality to write ROS 2 LaserScan messages that contain
 * data from a planar laser range-finder.
 */
class Ros2LaserScanMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 LaserScan message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Generate the buffers according to the LaserScan metadata.
     * @details
     * It allocates memory for the range and intensities data.
     *
     * @param[in] buffSize buffer size.
     */
    virtual void generateBuffers(const size_t buffSize) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets all fields in a ROS 2 LaserScan message.
     *
     * @param[in] azimuthRange Start (`angle_min`) and end (`angle_max`) angles of the scan in degrees.
     * @param[in] rotationRate Scan frequency in Hz (`1 / scan_time`).
     * @param[in] depthRange Minimum (`range_min`) and maximum (`range_max`) range values.
     * @param[in] horizontalResolution Angular distance (`angle_increment`) between measurements in degrees.
     * @param[in] horizontalFov Horizontal field of view (`360 * time_increment * ranges.size / scan_time`) in degrees.
     */
    virtual void writeData(const pxr::GfVec2f& azimuthRange,
                           const float& rotationRate,
                           const pxr::GfVec2f& depthRange,
                           float horizontalResolution,
                           float horizontalFov) = 0;

    /**
     * @brief Get the buffer (range data).
     * @details
     * Provides direct access to the underlying buffer containing the range data.
     *
     * @return Buffer.
     */
    std::vector<float>& getRangeData()
    {
        return m_rangeData;
    }

    /**
     * @brief Get the buffer (intensities data).
     * @details
     * Provides direct access to the underlying buffer containing the intensities data.
     *
     * @return Buffer.
     */
    std::vector<float>& getIntensitiesData()
    {
        return m_intensitiesData;
    }

protected:
    /**
     * @brief Buffer (range data).
     */
    std::vector<float> m_rangeData;

    /**
     * @brief Buffer (intensities data).
     */
    std::vector<float> m_intensitiesData;
};

/**
 * @class Ros2OdometryMessage
 * @brief Class implementing a `nav_msgs/msg/Odometry` message.
 * @details
 * Provides functionality to write ROS 2 Odometry messages that contain
 * position and velocity information in free space.
 */
class Ros2OdometryMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 Odometry message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the pose, twist, and child frame fields in a ROS 2 Odometry message.
     * Optionally transforms velocities to the robot's coordinate frame.
     *
     * @param[in] childFrame Frame id the pose points to.
     * @param[in] linearVelocity Linear velocity.
     * @param[in] angularVelocity Angular velocity.
     * @param[in] robotFront Front normalized vector.
     * @param[in] robotSide Side normalized vector.
     * @param[in] robotUp Up normalized vector.
     * @param[in] unitScale Unit scale of the stage.
     * @param[in] position Position.
     * @param[in] orientation Orientation.
     * @param[in] publishRawVelocities If enabled, raw velocities are published. Otherwise velocities are projected in
     * the robot frame before being published.
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
                           bool publishRawVelocities) = 0;
};

/**
 * @class Ros2PointCloudMessage
 * @brief Class implementing a `sensor_msgs/msg/PointCloud2` message.
 * @details
 * Provides functionality to write ROS 2 PointCloud2 messages that contain
 * point cloud data with an arbitrary number of fields per point.
 */
class Ros2PointCloudMessage : public Ros2Message
{
public:
    /**
     * @brief PointField type sizes. Maps from anonymous enums in sensor_msgs/msg/PointField to the actual size of the
     * type.
     */
    static constexpr std::array<size_t, 9> s_kPointFieldTypeSizes = { 0,
                                                                      sizeof(int8_t),
                                                                      sizeof(uint8_t),
                                                                      sizeof(int16_t),
                                                                      sizeof(uint16_t),
                                                                      sizeof(int32_t),
                                                                      sizeof(uint32_t),
                                                                      sizeof(float),
                                                                      sizeof(double) };

    /**
     * @brief Allocates and initializes the point cloud message buffer
     * @param[in] timeStamp Time in seconds
     * @param[in] frameId Frame ID for the point cloud
     * @param[in] bufferSize Size of the point cloud data buffer in bytes
     * @param[in] intensityPtr Pointer to the intensity data
     * @param[in] timestampPtr Pointer to the timestamp data
     * @param[in] emitterIdPtr Pointer to the emitter ID data
     * @param[in] channelIdPtr Pointer to the channel ID data
     * @param[in] materialIdPtr Pointer to the material ID data
     * @param[in] tickIdPtr Pointer to the tick ID data
     * @param[in] hitNormalPtr Pointer to the normal data
     * @param[in] velocityPtr Pointer to the velocity data
     * @param[in] objectIdPtr Pointer to the object ID data
     * @param[in] echoIdPtr Pointer to the echo ID data
     * @param[in] tickStatePtr Pointer to the tick states data
     * @param[in] radialVelocityMSPtr Pointer to the radial velocity data
     */
    virtual void generateBuffer(const double& timeStamp,
                                const std::string& frameId,
                                const size_t& bufferSize,
                                float* intensityPtr,
                                uint64_t* timestampPtr,
                                uint32_t* emitterIdPtr,
                                uint32_t* channelIdPtr,
                                uint32_t* materialIdPtr,
                                uint32_t* tickIdPtr,
                                pxr::GfVec3f* hitNormalPtr,
                                pxr::GfVec3f* velocityPtr,
                                uint32_t* objectIdPtr,
                                uint8_t* echoIdPtr,
                                uint8_t* tickStatePtr,
                                float* radialVelocityMSPtr) = 0;

    /**
     * @brief Get the pointer to the buffer (data).
     * @details
     * Provides direct access to the underlying buffer containing the point cloud data.
     *
     * @return Pointer to the buffer.
     */
    void* getBufferPtr()
    {
        return m_usePinnedBuffer ? m_bufferPinned.data() : m_buffer.data();
    }

    /**
     * @brief Get the total size (`width * point_step`) of the buffer, in bytes.
     * @details
     * Returns the total size of the point cloud buffer in bytes.
     *
     * @return Buffer size.
     */
    size_t getTotalBytes() const
    {
        return m_totalBytes;
    }

    /**
     * @brief Get the number of points.
     * @details
     * Returns the number of points in the point cloud.
     *
     * @return Number of points.
     */
    size_t getNumPoints() const
    {
        return m_numPoints;
    }

    /**
     * @brief Get the point step.
     * @details
     * Returns the point step in the point cloud.
     *
     * @return Point step.
     */
    size_t getPointStep() const
    {
        return m_pointStep;
    }

    /**
     * @brief Get the ordered list of metadata pointers and corresponding offsets.
     * @details
     * Returns the ordered list of metadata pointers, size of their type in bytes, and corresponding offsets.
     *
     * @return Ordered list of metadata pointers, size of their type in bytes, and corresponding offsets.
     */
    const std::vector<std::tuple<void*, size_t, size_t>>& getOrderedFields() const
    {
        return m_orderedFields;
    }

    /**
     * @brief Set whether to use the pinned buffer for the message data.
     */
    void setUsePinnedBuffer(bool usePinnedBuffer)
    {
        m_usePinnedBuffer = usePinnedBuffer;
    }

protected:
    /**
     * @brief Buffer (data) in pinned host memory.
     */
    isaacsim::core::includes::GenericBuffer m_bufferPinned;

    /**
     * @brief Buffer (data) in unpinned host memory.
     */
    std::vector<uint8_t> m_buffer;

    /**
     * @brief Buffer size.
     */
    size_t m_totalBytes{ 0 };

    /**
     * @brief Point step.
     */
    size_t m_pointStep{ 0 };

    /**
     * @brief Number of points.
     */
    size_t m_numPoints{ 0 };

    /**
     * @brief Ordered list of metadata pointers, size of their type in bytes, and corresponding offsets.
     */
    std::vector<std::tuple<void*, size_t, size_t>> m_orderedFields;

    /**
     * @brief Whether to use the pinned buffer for the message data.
     */
    bool m_usePinnedBuffer{ false };
};

/**
 * @class Ros2RawTfTreeMessage
 * @brief Class implementing a `tf2_msgs/msg/TFMessage` message with only one transform.
 * @details
 * Provides functionality to write ROS 2 TFMessage messages that contain
 * a single transform between coordinate frames.
 */
class Ros2RawTfTreeMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the transform data in a ROS 2 TFMessage message with a single transform.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     * @param[in] childFrame Frame ID of the child frame to which this transform points.
     * @param[in] translation Translation of child frame from header frame.
     * @param[in] rotation Rotation of child frame from header frame.
     */
    virtual void writeData(const double timeStamp,
                           const std::string& frameId,
                           const std::string& childFrame,
                           const pxr::GfVec3d& translation,
                           const pxr::GfQuatd& rotation) = 0;
};

/**
 * @class Ros2SemanticLabelMessage
 * @brief Class implementing a `std_msgs/msg/String` message for semantic label.
 * @details
 * Provides functionality to write ROS 2 String messages that contain
 * semantic label information.
 */
class Ros2SemanticLabelMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the string data in a ROS 2 String message.
     *
     * @param[in] data String data.
     */
    virtual void writeData(const std::string& data) = 0;
};

/**
 * @class Ros2TwistMessage
 * @brief Class implementing a `geometry_msgs/msg/Twist`.
 * @details
 * Provides functionality to read ROS 2 Twist messages that contain
 * linear and angular velocity information.
 */
class Ros2TwistMessage : public Ros2Message
{
public:
    /**
     * @brief Read the message field values.
     * @details
     * Extracts linear and angular velocity from a ROS 2 Twist message.
     *
     * @param[out] linearVelocity Linear velocity.
     * @param[out] angularVelocity Angular velocity.
     */
    virtual void readData(pxr::GfVec3d& linearVelocity, pxr::GfVec3d& angularVelocity) = 0;
};

/**
 * @class Ros2AckermannDriveStampedMessage
 * @brief Class implementing a `ackermann_msgs/msg/AckermannDriveStamped`.
 * @details
 * Provides functionality to read and write ROS 2 AckermannDriveStamped messages that contain
 * control commands for a vehicle with Ackermann steering.
 */
class Ros2AckermannDriveStampedMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message header.
     * @details
     * Sets the header fields in a ROS 2 AckermannDriveStamped message.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] frameId Transform frame with which this data is associated.
     */
    virtual void writeHeader(const double timeStamp, const std::string& frameId) = 0;

    /**
     * @brief Read the message field values.
     * @details
     * Extracts timestamp, frame ID, and Ackermann drive parameters from
     * a ROS 2 AckermannDriveStamped message.
     *
     * @param[out] timeStamp Time (seconds).
     * @param[out] frameId Transform frame with which this data is associated.
     * @param[out] steeringAngle Virtual angle.
     * @param[out] steeringAngleVelocity Rate of change.
     * @param[out] speed Forward speed.
     * @param[out] acceleration Acceleration.
     * @param[out] jerk Jerk.
     */
    virtual void readData(double& timeStamp,
                          std::string& frameId,
                          double& steeringAngle,
                          double& steeringAngleVelocity,
                          double& speed,
                          double& acceleration,
                          double& jerk) = 0;

    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the Ackermann drive parameters in a ROS 2 AckermannDriveStamped message.
     *
     * @param[in] steeringAngle Virtual angle.
     * @param[in] steeringAngleVelocity Rate of change.
     * @param[in] speed Forward speed.
     * @param[in] acceleration Acceleration.
     * @param[in] jerk Jerk.
     */
    virtual void writeData(const double& steeringAngle,
                           const double& steeringAngleVelocity,
                           const double& speed,
                           const double& acceleration,
                           const double& jerk) = 0;
};

/**
 * @class Ros2TfTreeMessage
 * @brief Class implementing a `tf2_msgs/msg/TFMessage` message.
 * @details
 * Provides functionality to read and write ROS 2 TFMessage messages that contain
 * multiple transforms between coordinate frames.
 */
class Ros2TfTreeMessage : public Ros2Message
{
public:
    /**
     * @brief Write the message field values from the given arguments.
     * @details
     * Sets the transforms in a ROS 2 TFMessage message with multiple transforms.
     *
     * @param[in] timeStamp Time (seconds).
     * @param[in] transforms Transforms.
     */
    virtual void writeData(const double& timeStamp, std::vector<TfTransformStamped>& transforms) = 0;

    /**
     * @brief Read the message field values.
     * @details
     * Extracts transform data from a ROS 2 TFMessage message.
     *
     * @param[out] transforms Transforms.
     */
    virtual void readData(std::vector<TfTransformStamped>& transforms) = 0;
};

}
}
}
