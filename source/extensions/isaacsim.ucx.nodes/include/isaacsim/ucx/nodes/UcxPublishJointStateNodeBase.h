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
#include <omni/graph/core/iComputeGraph.h>

#include <cstring>
#include <vector>

using omni::graph::core::GraphInstanceID;
using omni::graph::core::NodeObj;

namespace isaacsim::ucx::nodes
{

/**
 * @struct JointStateData
 * @brief Data structure for joint state message payload.
 * @details
 * Contains robot joint state data including positions, velocities, and efforts
 * for all joints in the articulation.
 */
struct JointStateData
{
    double timestamp; //!< Timestamp value in seconds
    int64_t numJoints; //!< Number of joints
    std::vector<double> positions; //!< Joint positions
    std::vector<double> velocities; //!< Joint velocities
    std::vector<double> efforts; //!< Joint efforts/forces
};

} // namespace isaacsim::ucx::nodes

/**
 * @class UCXPublishJointStateNodeBase
 * @brief Templated base class for UCX joint state publishing nodes.
 * @details
 * This template provides common functionality for publishing robot joint state data over UCX.
 * Derived classes implement message generation logic via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishJointStateNodeBase : public isaacsim::ucx::nodes::UcxNode
{
public:
    /**
     * @brief Reset the node state.
     */
    virtual void reset() override
    {
        UcxNode::reset();
    }

protected:
    /**
     * @brief Common compute logic for joint state publishing nodes.
     * @details
     * Handles listener initialization, connection checking, and message publishing.
     * Extracts data using extractData() and serializes using generateMessage().
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX listener
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for send request (0 = infinite)
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

        isaacsim::ucx::nodes::JointStateData data = extractData(db);
        return this->publishMessage(db, generateMessage(data), tag, timeoutMs);
    }

    /**
     * @brief Extract joint state data from input ports.
     * @details
     * Pure virtual function that derived classes must implement to read
     * joint data from the input ports and return it as JointStateData.
     *
     * @param[in] db Database accessor for node inputs
     * @return JointStateData Extracted joint state data
     */
    virtual isaacsim::ucx::nodes::JointStateData extractData(DatabaseT& db) = 0;

    /**
     * @brief Generate message from joint state data.
     * @details
     * Pure virtual function that derived classes must implement to serialize
     * joint state data into the appropriate message format.
     *
     * @param[in] data Joint state data to serialize
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::JointStateData& data) = 0;
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXPublishJointStateNodeBase<YourDatabase>
// 2. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 3. Implement virtual JointStateData extractData(DatabaseT& db) override
//    - Read joint data from input ports (jointPositions, jointVelocities, jointEfforts) and return as JointStateData
// 4. Implement virtual std::vector<uint8_t> generateMessage(const JointStateData& data) override
//    - Serialize the joint state data into message format
// 5. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag, timeoutMs)
// 6. See OgnUCXPublishJointState.cpp for examples
