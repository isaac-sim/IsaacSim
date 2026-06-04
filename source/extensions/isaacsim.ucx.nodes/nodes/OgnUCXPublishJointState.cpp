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

#include <flatbuffers/flatbuffers.h>
#include <isaacsim/ucx/nodes/UcxPublishJointStateNodeBase.h>

#include <OgnUCXPublishJointStateDatabase.h>
#include <joint_state_generated.h>
#include <time_generated.h>

using namespace isaacsim::ucx::nodes;

/**
 * @class OgnUCXPublishJointState
 * @brief OmniGraph node for publishing joint states via UCX.
 * @details
 * This node publishes robot joint state data over UCX using tagged communication.
 * It reads joint data from upstream input ports (e.g. connected from Isaac Read Joint State).
 */
class OgnUCXPublishJointState : public UCXPublishJointStateNodeBase<OgnUCXPublishJointStateDatabase>
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
        auto& state = OgnUCXPublishJointStateDatabase::sPerInstanceState<OgnUCXPublishJointState>(nodeObj, instanceId);
        state.reset();
    }

    /**
     * @brief Compute function - called when node is executed.
     * @details
     * Extracts inputs, gets the per-instance state, and delegates to the base class logic.
     * Triggers the execution output port upon successful execution.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @return bool True if execution succeeded, false otherwise
     */
    static bool compute(OgnUCXPublishJointStateDatabase& db)
    {
        auto& state = db.template perInstanceState<OgnUCXPublishJointState>();

        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        bool success = state.computeImpl(db, port, tag, timeoutMs);

        if (success)
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
        }

        return success;
    }

protected:
    /**
     * @brief Extract joint state data from input ports.
     *
     * @param[in] db Database accessor for node inputs
     * @return JointStateData Extracted joint state data
     */
    isaacsim::ucx::nodes::JointStateData extractData(OgnUCXPublishJointStateDatabase& db) override
    {
        isaacsim::ucx::nodes::JointStateData data;
        data.timestamp = db.inputs.timeStamp();
        auto positions = db.inputs.jointPositions();
        auto velocities = db.inputs.jointVelocities();
        auto efforts = db.inputs.jointEfforts();
        data.numJoints = static_cast<int64_t>(positions.size());
        data.positions.assign(positions.begin(), positions.end());
        data.velocities.assign(velocities.begin(), velocities.end());
        data.efforts.assign(efforts.begin(), efforts.end());
        return data;
    }

    /**
     * @brief Generate message from joint state data.
     * @details
     * Serializes joint state data as a FlatBuffers JointState message.
     *
     * @param[in] data Joint state data to serialize
     * @return std::vector<uint8_t> Serialized message data
     */
    std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::JointStateData& data) override
    {
        flatbuffers::FlatBufferBuilder builder;

        // Names (optional; JointStateData has no names)
        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>> names_fb = 0;

        auto makeTensorF32Vec = [&](const std::vector<double>& values)
        {
            std::vector<float> f32(values.begin(), values.end());
            auto data_fb = builder.CreateVector(reinterpret_cast<const uint8_t*>(f32.data()), sizeof(float) * f32.size());
            std::vector<int64_t> shape_vec = { static_cast<int64_t>(f32.size()) };
            auto shape_fb = builder.CreateVector(shape_vec);
            isaac::DLDataType dtype(isaac::DLDataTypeCode_kDLFloat, 32, 1);
            isaac::DLDevice device(isaac::DLDeviceType_kDLCPU, 0);
            uint32_t ndim = 1;
            std::vector<int64_t> strides_vec = { 1 };
            auto strides_fb = builder.CreateVector(strides_vec);
            return isaac::CreateTensor(builder, data_fb, shape_fb, &dtype, &device, ndim, strides_fb);
        };

        auto position_fb = makeTensorF32Vec(data.positions);
        auto velocity_fb = makeTensorF32Vec(data.velocities);
        auto effort_fb = makeTensorF32Vec(data.efforts);

        // Header (stamp is Time with time_ns)
        const int64_t time_ns = static_cast<int64_t>(data.timestamp * 1e9);
        auto stamp_fb = isaac::CreateTime(builder, time_ns, 0);
        auto header_fb = isaac::CreateHeader(builder, stamp_fb, builder.CreateString(""));

        // Pack JointState
        auto joint_state_fb = isaac::CreateJointState(builder, header_fb, names_fb, position_fb, velocity_fb,
                                                      0, // acceleration
                                                      // empty_tensor_fb,     // acceleration
                                                      effort_fb,
                                                      0 // temperature
                                                      // empty_tensor_fb      // temperature
        );

        builder.Finish(joint_state_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        size_t bufSize = builder.GetSize();
        return std::vector<uint8_t>(bufPtr, bufPtr + bufSize);
    }
};

REGISTER_OGN_NODE()
