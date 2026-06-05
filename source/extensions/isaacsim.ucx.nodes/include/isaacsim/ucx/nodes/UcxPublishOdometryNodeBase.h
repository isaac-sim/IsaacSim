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

#include <array>
#include <cstring>
#include <vector>

using omni::graph::core::GraphInstanceID;
using omni::graph::core::NodeObj;

namespace isaacsim::ucx::nodes
{

/**
 * @struct OdometryData
 * @brief Data structure for odometry message payload.
 * @details
 * Contains computed odometry data including relative pose and body-frame velocities/accelerations.
 * All values are in body frame relative to the starting pose.
 */
struct OdometryData
{
    double timestamp; //!< Timestamp value in seconds
    std::array<double, 3> position; //!< Relative position (x, y, z) in body frame
    std::array<double, 4> orientation; //!< Relative orientation quaternion (w, x, y, z)
    std::array<double, 3> linearVelocity; //!< Linear velocity (x, y, z) in body frame
    std::array<double, 3> angularVelocity; //!< Angular velocity (x, y, z) in body frame
    std::array<double, 3> linearAcceleration; //!< Linear acceleration (x, y, z) in body frame
    std::array<double, 3> angularAcceleration; //!< Angular acceleration (x, y, z) in body frame
};

} // namespace isaacsim::ucx::nodes

/**
 * @class UCXPublishOdometryNodeBase
 * @brief Templated base class for UCX odometry data publishing nodes.
 * @details
 * This template provides common functionality for publishing odometry data over UCX
 * with timeout support. Derived classes implement data extraction via extractData()
 * and message serialization via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishOdometryNodeBase : public isaacsim::ucx::nodes::UcxNode
{
public:
    /**
     * @brief Initialize the node instance.
     * @details
     * Default implementation - can be overridden by derived classes if needed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        // Default: no initialization needed
    }

protected:
    /**
     * @brief Common compute logic for odometry publishing nodes.
     * @details
     * Handles listener initialization, connection checking, and message publishing with timeout.
     * Extracts data using extractData() and serializes using generateMessage().
     * Sets execOut port on success.
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

        isaacsim::ucx::nodes::OdometryData data = extractData(db);
        bool success = this->publishMessage(db, generateMessage(data), tag, timeoutMs);

        if (success)
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
        }

        return success;
    }

    /**
     * @brief Extract odometry data from node inputs.
     * @details
     * Pure virtual function that derived classes must implement to extract
     * and compute odometry data from their specific database type.
     * This includes reading raw inputs, computing relative poses, and
     * transforming velocities to body frame.
     *
     * @param[in] db Database accessor for node inputs
     * @return OdometryData Computed odometry data
     */
    virtual isaacsim::ucx::nodes::OdometryData extractData(DatabaseT& db) = 0;

    /**
     * @brief Generate message from odometry data.
     * @details
     * Pure virtual function that derived classes must implement to serialize
     * odometry data into the appropriate message format.
     *
     * @param[in] data Odometry data to serialize
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::OdometryData& data) = 0;
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXPublishOdometryNodeBase<YourDatabase>
// 2. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 3. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db (port, tag, timeoutMs)
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag, timeoutMs)
// 4. Implement virtual OdometryData extractData(DatabaseT& db) override
//    - Read inputs, compute relative pose and body-frame velocities
// 5. Implement virtual std::vector<uint8_t> generateMessage(const OdometryData& data) override
//    - Serialize the odometry data into the message format
// 6. See OgnUCXPublishOdometry.cpp for examples
