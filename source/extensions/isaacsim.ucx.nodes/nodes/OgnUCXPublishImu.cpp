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


#if defined(__GNUC__) && !defined(__clang__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
#include <flatbuffers/flatbuffers.h>
#if defined(__GNUC__) && !defined(__clang__)
#    pragma GCC diagnostic pop
#endif

#include <isaacsim/ucx/nodes/UcxPublishImuNodeBase.hpp>

#include <OgnUCXPublishImuDatabase.h>
#include <imu_generated.h>

using namespace isaacsim::ucx::nodes;

/**
 * @class OgnUCXPublishImu
 * @brief OmniGraph node for publishing IMU data via UCX.
 * @details
 * This node publishes IMU sensor data over UCX using tagged communication.
 * It supports orientation, linear acceleration, and angular velocity data,
 * serialized as a FlatBuffers Imu message.
 */
class OgnUCXPublishImu : public UCXPublishImuNodeBase<OgnUCXPublishImuDatabase>
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
        auto& state = OgnUCXPublishImuDatabase::sPerInstanceState<OgnUCXPublishImu>(nodeObj, instanceId);
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
    static bool compute(OgnUCXPublishImuDatabase& db)
    {
        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        auto& state = db.template perInstanceState<OgnUCXPublishImu>();
        return state.computeImpl(db, port, tag, timeoutMs);
    }

protected:
    /**
     * @brief Generate message from IMU data.
     * @details
     * Serializes IMU data as a FlatBuffers Imu message. Orientation is stored
     * as a float32 tensor with shape [4] in order [w, x, y, z]. Angular velocity
     * and linear acceleration are stored as Vector3f structs. Fields absent from
     * the input (publish flags false) are omitted from the message.
     *
     * @param[in] data IMU data to serialize
     * @return std::vector<uint8_t> Serialized FlatBuffers message
     */
    std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::ImuData& data) override
    {
        flatbuffers::FlatBufferBuilder builder;

        holoscan::schema::Quaternionf orientation;
        auto flags = static_cast<holoscan::schema::ImuFlags>(0);
        if (data.publishOrientation)
        {
            orientation = holoscan::schema::Quaternionf(
                static_cast<float>(data.orientation[0]), static_cast<float>(data.orientation[1]),
                static_cast<float>(data.orientation[2]), static_cast<float>(data.orientation[3]));
            flags = holoscan::schema::ImuFlags_ORIENTATION_VALID;
        }

        // Angular velocity: Vector3f struct (x, y, z) in rad/s.
        holoscan::schema::Vector3f angularVel(0.0f, 0.0f, 0.0f);
        holoscan::schema::Vector3f* angularVelPtr = nullptr;
        if (data.publishAngularVelocity)
        {
            angularVel = holoscan::schema::Vector3f(static_cast<float>(data.angularVelocity[0]),
                                                    static_cast<float>(data.angularVelocity[1]),
                                                    static_cast<float>(data.angularVelocity[2]));
            angularVelPtr = &angularVel;
        }

        // Linear acceleration: Vector3f struct (x, y, z) in m/s^2.
        holoscan::schema::Vector3f linearAccel(0.0f, 0.0f, 0.0f);
        holoscan::schema::Vector3f* linearAccelPtr = nullptr;
        if (data.publishLinearAcceleration)
        {
            linearAccel = holoscan::schema::Vector3f(static_cast<float>(data.linearAcceleration[0]),
                                                     static_cast<float>(data.linearAcceleration[1]),
                                                     static_cast<float>(data.linearAcceleration[2]));
            linearAccelPtr = &linearAccel;
        }

        // Header
        const int64_t time_ns = static_cast<int64_t>(data.timestamp * 1e9);
        auto frame_id_fb = builder.CreateString(data.frameId);
        auto header_fb = holoscan::schema::CreateHeader(builder, 0, frame_id_fb, time_ns);

        auto imu_fb = holoscan::schema::CreateImu(builder, header_fb, &orientation, angularVelPtr, linearAccelPtr, flags);
        holoscan::schema::FinishImuBuffer(builder, imu_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        size_t bufSize = builder.GetSize();
        return std::vector<uint8_t>(bufPtr, bufPtr + bufSize);
    }
};

REGISTER_OGN_NODE()
