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

#include <isaacsim/ucx/core/UcxListenerRegistry.h>
#include <isaacsim/ucx/core/UcxUtils.h>
#include <isaacsim/ucx/nodes/UcxNode.h>
#include <omni/graph/core/CppWrappers.h>

#include <cstring>
#include <vector>

namespace isaacsim::ucx::nodes
{

/**
 * @struct JointCommandData
 * @brief Data structure for joint command message payload.
 * @details
 * Contains robot joint command data including positions, velocities, and efforts
 * for all joints in the articulation.
 */
struct JointCommandData
{
    double timestamp; //!< Timestamp value in seconds
    int64_t numJoints; //!< Number of joints
    std::vector<double> positionCommand; //!< Joint position commands
    std::vector<double> velocityCommand; //!< Joint velocity commands
    std::vector<double> effortCommand; //!< Joint effort commands
    bool valid; //!< Flag indicating if data was successfully parsed
};

/**
 * @brief Base class template for UCX joint state subscriber nodes.
 * @details
 * Provides common functionality for receiving joint command data via UCX.
 * Derived classes implement message parsing via parseMessage().
 * The base class handles setting outputs via setOutputs().
 *
 * @tparam DatabaseT The OmniGraph database type for the derived node
 */
template <typename DatabaseT>
class UCXSubscribeJointCommandNodeBase : public UcxNode
{
public:
    using NodeObj = omni::graph::core::NodeObj; ///< Alias for the OmniGraph node object type.
    using GraphInstanceID = omni::graph::core::GraphInstanceID; ///< Alias for the OmniGraph instance ID type.

    /**
     * @brief Initialize the node instance.
     * @details
     * Default implementation - derived classes should override to initialize state.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        // Default: no initialization needed
        // Derived classes should override and reserve buffer if needed
    }

    /**
     * @brief Reset the node state.
     * @details
     * Clears output arrays and resets the connection.
     */
    virtual void reset() override
    {
        m_receiveBuffer.clear();
        UcxNode::reset();
    }

protected:
    /**
     * @brief Compute implementation for joint state subscription.
     * @details
     * Receives joint command data via UCX and outputs it.
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX communication
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for receive request (0 = infinite)
     * @return bool True if execution succeeded, false otherwise
     */
    bool computeImpl(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        if (!this->ensureListenerReady(db, port))
        {
            return false;
        }

        if (!this->waitForConnection())
        {
            return true;
        }

        return receiveMessage(db, tag, timeoutMs);
    }

    /**
     * @brief Receive and process joint command message.
     * @details
     * Receives a message via UCX, parses it using parseMessage(), and sets
     * outputs using setOutputs(). If the request doesn't complete within
     * the timeout, it returns without error.
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds (0 = infinite)
     * @return bool True if receive succeeded, false otherwise
     */
    bool receiveMessage(DatabaseT& db, uint64_t tag, uint32_t timeoutMs)
    {
        try
        {
            const size_t maxBufferSize = 65536;
            m_receiveBuffer.resize(maxBufferSize);

            std::string errorMessage;
            auto result = this->m_listener->tagReceive(
                m_receiveBuffer.data(), m_receiveBuffer.size(), tag, 0xFFFFFFFFFFFFFFFF, errorMessage, timeoutMs);

            if (result == isaacsim::ucx::core::UcxReceiveResult::eTimedOut)
            {
                db.logWarning("Receive request timed out after %u ms: %s", timeoutMs, errorMessage.c_str());
                return true;
            }
            else if (result == isaacsim::ucx::core::UcxReceiveResult::eFailed)
            {
                db.logWarning("Receive request failed: %s", errorMessage.c_str());
                return true;
            }
            else if (result != isaacsim::ucx::core::UcxReceiveResult::eSuccess)
            {
                db.logWarning("Receive request returned unexpected result: %s", errorMessage.c_str());
                return true;
            }

            JointCommandData data = parseMessage(m_receiveBuffer);

            if (data.valid)
            {
                setOutputs(db, data);
            }

            return true;
        }
        catch (const std::exception& e)
        {
            db.logError("Exception during message receive: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Parse joint command message from raw bytes.
     * @details
     * Pure virtual function that derived classes must implement to parse
     * the raw message buffer into a JointCommandData structure.
     *
     * @param[in] buffer Raw message buffer
     * @return JointCommandData Parsed joint command data with valid flag set
     */
    virtual JointCommandData parseMessage(const std::vector<uint8_t>& buffer) = 0;

    /**
     * @brief Set node outputs from joint command data.
     * @details
     * Sets the database outputs from the parsed joint command data structure.
     *
     * @param[in,out] db Database accessor for node outputs
     * @param[in] data Joint command data to output
     */
    void setOutputs(DatabaseT& db, const JointCommandData& data)
    {
        db.outputs.positionCommand().resize(data.numJoints);
        db.outputs.velocityCommand().resize(data.numJoints);
        db.outputs.effortCommand().resize(data.numJoints);

        std::memcpy(db.outputs.positionCommand().data(), data.positionCommand.data(), sizeof(double) * data.numJoints);
        std::memcpy(db.outputs.velocityCommand().data(), data.velocityCommand.data(), sizeof(double) * data.numJoints);
        std::memcpy(db.outputs.effortCommand().data(), data.effortCommand.data(), sizeof(double) * data.numJoints);

        db.outputs.timeStamp() = data.timestamp;
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
    }

    std::vector<uint8_t> m_receiveBuffer; //!< Buffer for receiving messages
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXSubscribeJointCommandNodeBase<YourDatabase>
// 2. Implement static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Reserve receive buffer: state.m_receiveBuffer.reserve(65536)
// 3. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 4. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db (port, tag, timeoutMs)
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag, timeoutMs)
// 5. Implement virtual JointCommandData parseMessage(const std::vector<uint8_t>& buffer) override
//    - Parse raw bytes into JointCommandData structure
// 6. See OgnUCXSubscribeJointCommand.cpp for examples

} // namespace isaacsim::ucx::nodes
