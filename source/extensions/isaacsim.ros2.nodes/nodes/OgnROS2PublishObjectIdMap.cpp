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

#include <isaacsim/ros2/core/Ros2Node.h>
#include <nlohmann/json.hpp>

#include <OgnROS2PublishObjectIdMapDatabase.h>
#include <algorithm>
#include <cstring>
#include <vector>

using namespace isaacsim::ros2::core;

class OgnROS2PublishObjectIdMap : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishObjectIdMapDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishObjectIdMap>();

        // Spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
            const GraphContextObj& context = db.abi_context();
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!state.initializeNodeHandle(
                    std::string(nodeObj.iNode->getPrimPath(nodeObj)),
                    collectNamespace(db.inputs.nodeNamespace(),
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)))),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Publisher was not valid, create a new one
        if (!state.m_publisher)
        {
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            state.m_message = state.m_factory->createSemanticLabelMessage();

            Ros2QoSProfile qos;
            const std::string& qosProfile = db.inputs.qosProfile();
            if (qosProfile.empty())
            {
                qos.depth = db.inputs.queueSize();
            }
            else
            {
                if (!jsonToRos2QoSProfile(qos, qosProfile))
                {
                    return false;
                }
            }

            state.m_publisher = state.m_factory->createPublisher(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);
            return true;
        }

        return state.publishObjectIdMap(db);
    }

    bool publishObjectIdMap(OgnROS2PublishObjectIdMapDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishObjectIdMap>();

        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        const size_t dataSize = db.inputs.bufferSize();

        if (dataSize > 0 && db.inputs.dataPtr() != 0)
        {
            // Decode the stableIdMap buffer into a mapping of object IDs to prim paths
            // Format: sequence of entries (6 uint32s each), followed by number of entries (1 uint32)
            const auto* data = reinterpret_cast<const uint8_t*>(db.inputs.dataPtr());

            // Check if buffer has changed - if so, update cache and regenerate JSON
            if (state.m_cachedBuffer.size() != dataSize || std::memcmp(state.m_cachedBuffer.data(), data, dataSize) != 0)
            {
                state.m_cachedBuffer.assign(data, data + dataSize);
                state.m_cachedJson["id_to_labels"] = _generateIdToLabels(data, dataSize);
            }
        }

        // Update timestamp (always, regardless of buffer change)
        state.m_cachedJson["time_stamp"] = {};
        const auto result =
            std::div(static_cast<int64_t>(db.inputs.timeStamp() * 1e9), static_cast<int64_t>(1000000000L));
        if (result.rem >= 0)
        {
            state.m_cachedJson["time_stamp"]["sec"] = static_cast<std::int32_t>(result.quot);
            state.m_cachedJson["time_stamp"]["nanosec"] = static_cast<std::uint32_t>(result.rem);
        }
        else
        {
            state.m_cachedJson["time_stamp"]["sec"] = static_cast<std::int32_t>(result.quot - 1);
            state.m_cachedJson["time_stamp"]["nanosec"] = static_cast<std::uint32_t>(1000000000L + result.rem);
        }

        state.m_message->writeData(state.m_cachedJson.dump());
        state.m_publisher.get()->publish(state.m_message->getPtr());

        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnROS2PublishObjectIdMapDatabase::sPerInstanceState<OgnROS2PublishObjectIdMap>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        m_publisher.reset(); // This should be reset before we reset the handle.
        m_cachedBuffer.clear();
        m_cachedJson = nlohmann::json();
        Ros2Node::reset();
    }

private:
    nlohmann::json _generateIdToLabels(const uint8_t* data, size_t dataSize)
    {
        nlohmann::json idMapping;

        // Read the number of entries from the last 4 bytes
        uint32_t numEntries = 0;
        if (dataSize < 4)
        {
            return idMapping;
        }

        std::memcpy(&numEntries, data + dataSize - 4, 4);
        const size_t entrySize = 6 * sizeof(uint32_t); // 6 uint32s per entry

        for (uint32_t i = 0; i < numEntries; ++i)
        {
            const size_t offset = i * entrySize;
            if (offset + entrySize > dataSize)
            {
                break;
            }

            // Read entry data (6 uint32s)
            uint32_t entry[6];
            std::memcpy(entry, data + offset, entrySize);

            // Extract stable ID from first 4 uint32s
            uint32_t stableId[4];
            std::memcpy(stableId, entry, 4 * sizeof(uint32_t));

            // Convert 128-bit stable ID to decimal string (little endian)
            // We need to interpret the 16 bytes as a single 128-bit integer in little-endian order
            // Since C++ doesn't have native 128-bit integer support, we use manual multi-precision arithmetic

            // Convert to array of bytes for multi-precision arithmetic
            uint8_t bytes[16];
            std::memcpy(bytes, stableId, 16);

            // Convert 128-bit integer to decimal string using manual division
            std::string result;
            bool allZero = true;

            // Check if all bytes are zero
            for (int i = 0; i < 16; ++i)
            {
                if (bytes[i] != 0)
                {
                    allZero = false;
                    break;
                }
            }

            if (allZero)
            {
                result = "0";
            }
            else
            {
                // Divide by 10 repeatedly to get decimal digits
                std::vector<uint8_t> num(bytes, bytes + 16);

                while (true)
                {
                    // Check if number is zero
                    bool isZero = true;
                    for (size_t i = 0; i < num.size(); ++i)
                    {
                        if (num[i] != 0)
                        {
                            isZero = false;
                            break;
                        }
                    }

                    if (isZero)
                    {
                        break;
                    }

                    // Divide by 10 and get remainder
                    uint32_t remainder = 0;
                    for (int i = static_cast<int>(num.size()) - 1; i >= 0; --i)
                    {
                        uint32_t current = remainder * 256 + num[i];
                        num[i] = current / 10;
                        remainder = current % 10;
                    }

                    // Append digit to result
                    result = static_cast<char>('0' + remainder) + result;
                }
            }

            std::string idStream = result;

            // Extract label using offset and length
            uint32_t labelLength = entry[4];
            uint32_t labelOffset = entry[5];

            if (labelOffset + labelLength <= dataSize)
            {
                std::string label(reinterpret_cast<const char*>(data + labelOffset), labelLength);
                // Remove trailing null characters
                label.erase(std::find(label.begin(), label.end(), '\0'), label.end());

                idMapping[idStream] = label;
            }
        }

        return idMapping;
    }

    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2SemanticLabelMessage> m_message = nullptr;
    std::vector<uint8_t> m_cachedBuffer;
    nlohmann::json m_cachedJson;
};

REGISTER_OGN_NODE()
