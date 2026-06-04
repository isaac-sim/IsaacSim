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
#include <isaacsim/ucx/nodes/UcxSubscribeJointCommandNodeBase.h>

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
        state.m_receiveBuffer.reserve(65536); // Reserve 64KB for receive buffer
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
        auto* message = isaac::GetJointCommand(buffer.data());

        JointCommandData data;
        data.valid = false;
        if (!message || !message->header() || !message->header()->stamp() || !message->position() ||
            !message->position()->shape() || !message->velocity() || !message->effort())
        {
            return data;
        }
        data.timestamp = static_cast<double>(message->header()->stamp()->time_ns()) / 1e9;
        data.numJoints = message->position()->shape()->Get(0);

        data.positionCommand.resize(data.numJoints);
        data.velocityCommand.resize(data.numJoints);
        data.effortCommand.resize(data.numJoints);

        auto copyTensor = [&](std::vector<double>& dst, const isaac::Tensor* tensor) -> bool
        {
            if (!tensor->data() || tensor->data()->size() < static_cast<size_t>(data.numJoints) * sizeof(float))
            {
                return false;
            }
            const float* src = reinterpret_cast<const float*>(tensor->data()->data());
            for (int64_t i = 0; i < data.numJoints; ++i)
            {
                dst[i] = static_cast<double>(src[i]);
            }
            return true;
        };

        if (!copyTensor(data.positionCommand, message->position()) ||
            !copyTensor(data.velocityCommand, message->velocity()) || !copyTensor(data.effortCommand, message->effort()))
        {
            return data; // data.valid remains false
        }

        data.valid = true;
        return data;
    }
};

REGISTER_OGN_NODE()
