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
#include <isaacsim/ucx/nodes/UcxSubscribeJointCommandNodeBase.hpp>

#include <OgnUCXSubscribeJointCommandDatabase.h>
#include <joint_command_generated.h>

using namespace isaacsim::ucx::nodes;

/**
 * @class OgnUCXSubscribeJointCommand
 * @brief OmniGraph node for subscribing to joint state commands via UCX.
 * @details
 * This node receives joint command data over UCX using tagged communication.
 * It outputs position, velocity, and effort commands for robot joints.
 */
class OgnUCXSubscribeJointCommand : public UCXSubscribeJointCommandNodeBase<OgnUCXSubscribeJointCommandDatabase>
{
public:
    /**
     * @brief Initialize the node instance.
     * @details
     * Sets up the node state and reserves buffer for receiving messages.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnUCXSubscribeJointCommandDatabase::sPerInstanceState<OgnUCXSubscribeJointCommand>(nodeObj, instanceId);
        state.m_receive.reserveBuffer(OgnUCXSubscribeJointCommand::s_kReceiveBufferSize);
    }

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
        auto& state =
            OgnUCXSubscribeJointCommandDatabase::sPerInstanceState<OgnUCXSubscribeJointCommand>(nodeObj, instanceId);
        state.reset();
    }

    /**
     * @brief Compute function - called when node is executed.
     * @details
     * Receives joint command data via UCX and outputs it.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @return bool True if execution succeeded, false otherwise
     */
    static bool compute(OgnUCXSubscribeJointCommandDatabase& db)
    {
        auto& state = db.template perInstanceState<OgnUCXSubscribeJointCommand>();

        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        return state.computeImpl(db, port, tag, timeoutMs);
    }

protected:
    /**
     * @brief Parse joint command message from raw bytes.
     * @details
     * Deserializes the FlatBuffers-encoded JointCommand message into a JointCommandData structure.
     *
     * @param[in] buffer Raw message buffer
     * @return JointCommandData Parsed joint command data
     */
    JointCommandData parseMessage(const std::vector<uint8_t>& buffer) override
    {
        JointCommandData data{};
        flatbuffers::Verifier verifier(buffer.data(), buffer.size());
        if (!isaac::VerifyJointCommandBuffer(verifier))
            return data;
        const auto* message = isaac::GetJointCommand(buffer.data());
        if (!message->header())
            return data;
        data.timestamp = static_cast<double>(message->header()->capture_timestamp_ns()) / 1e9;

        auto copyTensor = [&](std::vector<double>& destination, const holoscan::schema::Tensor* tensor,
                              bool optional = false) -> bool
        {
            if (!tensor || !tensor->data() || !tensor->shape() || tensor->ndim() != 1 || tensor->shape()->size() != 1 ||
                (tensor->shape()->Get(0) < 0 || (!optional && tensor->shape()->Get(0) == 0)) || !tensor->dtype() ||
                tensor->dtype()->code() != holoscan::schema::DLDataTypeCode_kDLFloat || tensor->dtype()->bits() != 32 ||
                tensor->dtype()->lanes() != 1 || !tensor->device() ||
                tensor->device()->device_type() != holoscan::schema::DLDeviceType_kDLCPU ||
                (tensor->strides() && (tensor->strides()->size() != 1 || tensor->strides()->Get(0) != 1)))
                return false;
            const auto count = static_cast<uint64_t>(tensor->shape()->Get(0));
            if (count > 4096 || tensor->data()->size() != count * sizeof(float))
                return false;
            destination.resize(count);
            for (size_t i = 0; i < count; ++i)
            {
                float value;
                std::memcpy(&value, tensor->data()->data() + i * sizeof(float), sizeof(value));
                destination[i] = value;
            }
            return true;
        };
        if (!copyTensor(data.positionCommand, message->position()) ||
            !copyTensor(data.velocityCommand, message->velocity()) || !copyTensor(data.effortCommand, message->effort()) ||
            (message->kp() && !copyTensor(data.stiffness, message->kp(), true)) ||
            (message->kd() && !copyTensor(data.damping, message->kd(), true)))
            return data;
        data.numJoints = data.positionCommand.size();
        if (data.velocityCommand.size() != data.positionCommand.size() ||
            data.effortCommand.size() != data.positionCommand.size() ||
            (!data.stiffness.empty() && data.stiffness.size() != data.positionCommand.size()) ||
            (!data.damping.empty() && data.damping.size() != data.positionCommand.size()))
            return data;
        if (message->name())
        {
            if (message->name()->size() && message->name()->size() != data.positionCommand.size())
                return data;
        }

        data.valid = true;
        return data;
    }
};

REGISTER_OGN_NODE()
