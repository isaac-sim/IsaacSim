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
 * @struct ClockData
 * @brief Data structure for clock message payload.
 * @details
 * Contains the timestamp data to be published over UCX.
 */
struct ClockData
{
    double timestamp; //!< Timestamp value in seconds
};

} // namespace isaacsim::ucx::nodes

/**
 * @class UCXPublishClockNodeBase
 * @brief Templated base class for UCX clock publishing nodes.
 * @details
 * This template provides common functionality for publishing clock data over UCX.
 * Derived classes implement data extraction logic via extractData().
 * The base class handles message serialization via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishClockNodeBase : public isaacsim::ucx::nodes::UcxNode
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
     * @brief Common compute logic for clock publishing nodes.
     * @details
     * Handles listener initialization, connection checking, and message publishing with timeout.
     * Extracts data using the derived class's extractData() and serializes using generateMessage().
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

        isaacsim::ucx::nodes::ClockData data = extractData(db);
        return this->publishMessage(db, generateMessage(data), tag, timeoutMs);
    }

    /**
     * @brief Extract clock data from node inputs.
     * @details
     * Reads the timestamp from the database inputs. Can be overridden by
     * derived classes if different extraction logic is needed.
     *
     * @param[in] db Database accessor for node inputs
     * @return ClockData Extracted clock data
     */
    virtual isaacsim::ucx::nodes::ClockData extractData(DatabaseT& db)
    {
        return isaacsim::ucx::nodes::ClockData{ db.inputs.timeStamp() };
    }

    /**
     * @brief Generate message from clock data.
     * @details
     * Pure virtual function that derived classes must implement to serialize
     * clock data into the appropriate message format.
     *
     * @param[in] data Clock data to serialize
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::ClockData& data) = 0;
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXPublishClockNodeBase<YourDatabase>
// 2. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 3. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db (port, tag, timeoutMs)
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag, timeoutMs)
// 4. Implement virtual std::vector<uint8_t> generateMessage(const ClockData& data) override
//    - Serialize the clock data into the message format
// 5. Optionally override extractData() if different extraction logic is needed
// 6. See OgnUCXPublishClock.cpp for examples
