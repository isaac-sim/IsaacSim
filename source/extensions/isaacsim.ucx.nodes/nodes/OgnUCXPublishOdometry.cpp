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

#include <carb/Types.h>

#include <flatbuffers/flatbuffers.h>
#include <isaacsim/core/includes/Math.h>
#include <isaacsim/ucx/nodes/UcxPublishOdometryNodeBase.h>

#include <OgnUCXPublishOdometryDatabase.h>
#include <odometry_generated.h>

using isaacsim::core::includes::math::operator*; // NOLINT(misc-unused-using-decls)

/**
 * @class OgnUCXPublishOdometry
 * @brief OmniGraph node for publishing odometry data via UCX.
 * @details
 * This node takes position, orientation, and velocity inputs and publishes odometry data
 * (pose and velocities relative to starting position) over UCX using tagged communication.
 * Messages are serialized as FlatBuffers Odometry messages.
 */
class OgnUCXPublishOdometry : public UCXPublishOdometryNodeBase<OgnUCXPublishOdometryDatabase>
{
public:
    /**
     * @brief Release the node instance.
     * @details
     * Cleans up resources when the node instance is destroyed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnUCXPublishOdometryDatabase::sPerInstanceState<OgnUCXPublishOdometry>(nodeObj, instanceId);
        state.reset();
    }

    /**
     * @brief Compute function - called when node is executed.
     * @details
     * Extracts inputs, gets the per-instance state, and delegates to the base class logic.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @return bool True if execution succeeded, false otherwise
     */
    static bool compute(OgnUCXPublishOdometryDatabase& db)
    {
        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        auto& state = db.template perInstanceState<OgnUCXPublishOdometry>();
        return state.computeImpl(db, port, tag, timeoutMs);
    }

protected:
    /**
     * @brief Extract odometry data from node inputs.
     * @details
     * Takes position, orientation, and velocity inputs, computes odometry relative
     * to starting pose, and returns the computed data.
     *
     * @param[in] db Database accessor for node inputs
     * @return OdometryData Computed odometry data
     */
    isaacsim::ucx::nodes::OdometryData extractData(OgnUCXPublishOdometryDatabase& db) override
    {
        isaacsim::ucx::nodes::OdometryData data;
        data.timestamp = db.inputs.timeStamp();

        // Get direct inputs from node
        auto& position = db.inputs.position();
        auto& orientation = db.inputs.orientation();
        auto& linearVelocity = db.inputs.linearVelocity();
        auto& angularVelocity = db.inputs.angularVelocity();

        // Current world pose and velocities
        float currentPosX = static_cast<float>(position[0]);
        float currentPosY = static_cast<float>(position[1]);
        float currentPosZ = static_cast<float>(position[2]);

        // Orientation is in IJKR format (x, y, z, w)
        float currentQuatX = static_cast<float>(orientation.GetImaginary()[0]);
        float currentQuatY = static_cast<float>(orientation.GetImaginary()[1]);
        float currentQuatZ = static_cast<float>(orientation.GetImaginary()[2]);
        float currentQuatW = static_cast<float>(orientation.GetReal());

        float linearVelX = static_cast<float>(linearVelocity[0]);
        float linearVelY = static_cast<float>(linearVelocity[1]);
        float linearVelZ = static_cast<float>(linearVelocity[2]);

        float angularVelX = static_cast<float>(angularVelocity[0]);
        float angularVelY = static_cast<float>(angularVelocity[1]);
        float angularVelZ = static_cast<float>(angularVelocity[2]);

        // Initialize starting pose on first frame
        if (m_firstFrame)
        {
            m_startingPosition[0] = currentPosX;
            m_startingPosition[1] = currentPosY;
            m_startingPosition[2] = currentPosZ;
            m_startingOrientation[0] = currentQuatX;
            m_startingOrientation[1] = currentQuatY;
            m_startingOrientation[2] = currentQuatZ;
            m_startingOrientation[3] = currentQuatW;
        }

        // Compute relative position (world frame displacement)
        float deltaPosX = currentPosX - m_startingPosition[0];
        float deltaPosY = currentPosY - m_startingPosition[1];
        float deltaPosZ = currentPosZ - m_startingPosition[2];

        // Transform displacement to starting body frame
        carb::Float4 startingQuatInv = isaacsim::core::includes::math::inverse(carb::Float4{
            m_startingOrientation[0], m_startingOrientation[1], m_startingOrientation[2], m_startingOrientation[3] });

        carb::Float3 relativePosition =
            isaacsim::core::includes::math::rotate(startingQuatInv, carb::Float3{ deltaPosX, deltaPosY, deltaPosZ });

        // Compute relative orientation (current * starting^-1)
        carb::Float4 currentQuat = { currentQuatX, currentQuatY, currentQuatZ, currentQuatW };
        carb::Float4 relativeQuat = currentQuat * startingQuatInv;

        // Get velocities (world frame)
        carb::Float3 linearVelocityWorld = { linearVelX, linearVelY, linearVelZ };
        carb::Float3 angularVelocityWorld = { angularVelX, angularVelY, angularVelZ };

        // Transform velocities to body frame
        carb::Float4 currentQuatInv = isaacsim::core::includes::math::inverse(currentQuat);
        carb::Float3 linearVelocityBody = isaacsim::core::includes::math::rotate(currentQuatInv, linearVelocityWorld);
        carb::Float3 angularVelocityBody = isaacsim::core::includes::math::rotate(currentQuatInv, angularVelocityWorld);

        // Compute accelerations (body frame)
        carb::Float3 linearAcceleration = { 0.0f, 0.0f, 0.0f };
        carb::Float3 angularAcceleration = { 0.0f, 0.0f, 0.0f };

        if (m_firstFrame)
        {
            m_firstFrame = false;
        }
        else
        {
            // Simple finite difference
            const float dt = 0.016f; // Assume ~60Hz, could be improved with actual time tracking
            linearAcceleration.x = (linearVelocityBody.x - m_prevLinearVelocity.x) / dt;
            linearAcceleration.y = (linearVelocityBody.y - m_prevLinearVelocity.y) / dt;
            linearAcceleration.z = (linearVelocityBody.z - m_prevLinearVelocity.z) / dt;

            angularAcceleration.x = (angularVelocityBody.x - m_prevAngularVelocity.x) / dt;
            angularAcceleration.y = (angularVelocityBody.y - m_prevAngularVelocity.y) / dt;
            angularAcceleration.z = (angularVelocityBody.z - m_prevAngularVelocity.z) / dt;
        }

        // Store for next frame
        m_prevLinearVelocity = linearVelocityBody;
        m_prevAngularVelocity = angularVelocityBody;

        // Fill output data structure
        data.position = { static_cast<double>(relativePosition.x), static_cast<double>(relativePosition.y),
                          static_cast<double>(relativePosition.z) };
        data.orientation = { static_cast<double>(relativeQuat.w), static_cast<double>(relativeQuat.x),
                             static_cast<double>(relativeQuat.y), static_cast<double>(relativeQuat.z) };
        data.linearVelocity = { static_cast<double>(linearVelocityBody.x), static_cast<double>(linearVelocityBody.y),
                                static_cast<double>(linearVelocityBody.z) };
        data.angularVelocity = { static_cast<double>(angularVelocityBody.x), static_cast<double>(angularVelocityBody.y),
                                 static_cast<double>(angularVelocityBody.z) };
        data.linearAcceleration = { static_cast<double>(linearAcceleration.x), static_cast<double>(linearAcceleration.y),
                                    static_cast<double>(linearAcceleration.z) };
        data.angularAcceleration = { static_cast<double>(angularAcceleration.x),
                                     static_cast<double>(angularAcceleration.y),
                                     static_cast<double>(angularAcceleration.z) };

        return data;
    }

    /**
     * @brief Generate message from odometry data.
     * @details
     * Serializes odometry data as a FlatBuffers Odometry message. Position and orientation
     * are encoded in a PoseWithCovariance table (covariance omitted). Linear and angular
     * velocities are encoded in a TwistWithCovariance table (covariance omitted).
     * All tensors use float32 with shape [N].
     *
     * @param[in] data Odometry data to serialize
     * @return std::vector<uint8_t> Serialized FlatBuffers message
     */
    std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::OdometryData& data) override
    {
        flatbuffers::FlatBufferBuilder builder;

        auto makeTensorF32 = [&](const double* values, size_t count)
        {
            std::vector<float> f32(count);
            for (size_t i = 0; i < count; ++i)
            {
                f32[i] = static_cast<float>(values[i]);
            }
            auto data_fb = builder.CreateVector(reinterpret_cast<const uint8_t*>(f32.data()), count * sizeof(float));
            std::vector<int64_t> shape = { static_cast<int64_t>(count) };
            auto shape_fb = builder.CreateVector(shape);
            isaac::DLDataType dtype(isaac::DLDataTypeCode_kDLFloat, 32, 1);
            isaac::DLDevice device(isaac::DLDeviceType_kDLCPU, 0);
            std::vector<int64_t> strides = { 1 };
            auto strides_fb = builder.CreateVector(strides);
            return isaac::CreateTensor(builder, data_fb, shape_fb, &dtype, &device, 1, strides_fb);
        };

        // Pose: position (x, y, z) and orientation (w, x, y, z)
        auto position_fb = makeTensorF32(data.position.data(), 3);
        auto orientation_fb = makeTensorF32(data.orientation.data(), 4);
        auto pose_fb = isaac::CreatePose(builder, position_fb, orientation_fb);
        auto pose_cov_fb = isaac::CreatePoseWithCovariance(builder, pose_fb);

        // Twist: linear velocity (x, y, z) and angular velocity (x, y, z)
        auto linear_fb = makeTensorF32(data.linearVelocity.data(), 3);
        auto angular_fb = makeTensorF32(data.angularVelocity.data(), 3);
        auto twist_fb = isaac::CreateTwist(builder, linear_fb, angular_fb);
        auto twist_cov_fb = isaac::CreateTwistWithCovariance(builder, twist_fb);

        // Header
        const int64_t time_ns = static_cast<int64_t>(data.timestamp * 1e9);
        auto frame_id_fb = builder.CreateString("");
        auto stamp_fb = isaac::CreateTime(builder, time_ns, 0);
        auto header_fb = isaac::CreateHeader(builder, stamp_fb, frame_id_fb);

        auto odometry_fb = isaac::CreateOdometry(builder, header_fb, pose_cov_fb, twist_cov_fb);
        builder.Finish(odometry_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        size_t bufSize = builder.GetSize();
        return std::vector<uint8_t>(bufPtr, bufPtr + bufSize);
    }

private:
    float m_startingPosition[3] = { 0.0f, 0.0f, 0.0f }; //!< Starting position for odometry reference
    float m_startingOrientation[4] = { 0.0f, 0.0f, 0.0f, 1.0f }; //!< Starting orientation (x, y, z, w)
    carb::Float3 m_prevLinearVelocity = { 0.0f, 0.0f, 0.0f }; //!< Previous linear velocity for acceleration calculation
    carb::Float3 m_prevAngularVelocity = { 0.0f, 0.0f, 0.0f }; //!< Previous angular velocity for acceleration
                                                               //!< calculation
    bool m_firstFrame = true; //!< Flag indicating first frame to initialize starting pose
};

REGISTER_OGN_NODE()
