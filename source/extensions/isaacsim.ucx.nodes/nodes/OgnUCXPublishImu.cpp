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

#include <isaacsim/ucx/nodes/UcxPublishImuNodeBase.h>

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

        auto makeTensorF32 = [&](const float* values, size_t count)
        {
            auto data_fb = builder.CreateVector(reinterpret_cast<const uint8_t*>(values), count * sizeof(float));
            std::vector<int64_t> shape = { static_cast<int64_t>(count) };
            auto shape_fb = builder.CreateVector(shape);
            isaac::DLDataType dtype(isaac::DLDataTypeCode_kDLFloat, 32, 1);
            isaac::DLDevice device(isaac::DLDeviceType_kDLCPU, 0);
            std::vector<int64_t> strides = { 1 };
            auto strides_fb = builder.CreateVector(strides);
            return isaac::CreateTensor(builder, data_fb, shape_fb, &dtype, &device, 1, strides_fb);
        };

        // Orientation tensor: float32, shape [4], order [w, x, y, z].
        // ImuData orientation is stored as (x, y, z, w); reorder to (w, x, y, z).
        flatbuffers::Offset<isaac::Tensor> orientation_fb = 0;
        if (data.publishOrientation)
        {
            float orientData[4] = {
                static_cast<float>(data.orientation[3]), // w
                static_cast<float>(data.orientation[0]), // x
                static_cast<float>(data.orientation[1]), // y
                static_cast<float>(data.orientation[2]) // z
            };
            orientation_fb = makeTensorF32(orientData, 4);
        }

        // Angular velocity: Vector3f struct (x, y, z) in rad/s.
        isaac::Vector3f angularVel(0.0f, 0.0f, 0.0f);
        isaac::Vector3f* angularVelPtr = nullptr;
        if (data.publishAngularVelocity)
        {
            angularVel = isaac::Vector3f(static_cast<float>(data.angularVelocity[0]),
                                         static_cast<float>(data.angularVelocity[1]),
                                         static_cast<float>(data.angularVelocity[2]));
            angularVelPtr = &angularVel;
        }

        // Linear acceleration: Vector3f struct (x, y, z) in m/s^2.
        isaac::Vector3f linearAccel(0.0f, 0.0f, 0.0f);
        isaac::Vector3f* linearAccelPtr = nullptr;
        if (data.publishLinearAcceleration)
        {
            linearAccel = isaac::Vector3f(static_cast<float>(data.linearAcceleration[0]),
                                          static_cast<float>(data.linearAcceleration[1]),
                                          static_cast<float>(data.linearAcceleration[2]));
            linearAccelPtr = &linearAccel;
        }

        // Header
        const int64_t time_ns = static_cast<int64_t>(data.timestamp * 1e9);
        auto frame_id_fb = builder.CreateString(data.frameId);
        auto stamp_fb = isaac::CreateTime(builder, time_ns, 0);
        auto header_fb = isaac::CreateHeader(builder, stamp_fb, frame_id_fb);

        auto imu_fb = isaac::CreateImu(builder, header_fb, orientation_fb, angularVelPtr, linearAccelPtr);
        builder.Finish(imu_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        size_t bufSize = builder.GetSize();
        return std::vector<uint8_t>(bufPtr, bufPtr + bufSize);
    }
};

REGISTER_OGN_NODE()
