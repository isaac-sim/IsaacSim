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
#include <isaacsim/ucx/nodes/UcxPublishClockNodeBase.h>

#include <OgnUCXPublishClockDatabase.h>
#include <time_generated.h>

/**
 * @class OgnUCXPublishClock
 * @brief OmniGraph node for publishing clock messages via UCX.
 * @details
 * This node publishes timestamp data over UCX using tagged communication.
 * It uses the UCXPublishClockNodeBase template to minimize code duplication.
 */
class OgnUCXPublishClock : public UCXPublishClockNodeBase<OgnUCXPublishClockDatabase>
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
        auto& state = OgnUCXPublishClockDatabase::sPerInstanceState<OgnUCXPublishClock>(nodeObj, instanceId);
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
    static bool compute(OgnUCXPublishClockDatabase& db)
    {
        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        auto& state = db.template perInstanceState<OgnUCXPublishClock>();
        bool success = state.computeImpl(db, port, tag, timeoutMs);

        if (success)
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
        }

        return success;
    }

protected:
    /**
     * @brief Generate message from clock data.
     * @details
     * Serializes the clock data as a FlatBuffers Time message (time_ns).
     *
     * @param[in] data Clock data to serialize
     * @return std::vector<uint8_t> Serialized message data
     */
    std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::ClockData& data) override
    {
        flatbuffers::FlatBufferBuilder builder;
        const int64_t time_ns = static_cast<int64_t>(data.timestamp * 1e9);
        auto time_message = isaac::CreateTime(builder, time_ns, 0);
        builder.Finish(time_message);
        const uint8_t* buffer = builder.GetBufferPointer();
        const size_t size = builder.GetSize();
        return std::vector<uint8_t>(buffer, buffer + size);
    }
};

REGISTER_OGN_NODE()
