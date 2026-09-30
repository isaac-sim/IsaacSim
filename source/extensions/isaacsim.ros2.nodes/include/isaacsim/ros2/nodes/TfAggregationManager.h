// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ros2/core/Ros2Factory.hpp>
#include <isaacsim/ros2/core/Ros2QoS.hpp>
#include <isaacsim/ros2/core/Ros2Types.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace nodes
{

/**
 * @brief Aggregates TF submissions from multiple OGN publisher nodes into one ROS 2 TF publisher per topic group.
 *
 * Topic groups are keyed by ROS context, topic name, static/dynamic mode, and QoS profile. Dynamic groups are flushed
 * by the plugin's stage update node after OmniGraph evaluation. Manual OmniGraph evaluation callers can explicitly call
 * flushAll() if they need deterministic draining outside normal Kit playback.
 */
class TfAggregationManager
{
public:
    /** @brief Opaque identifier for one registered TF contributor. */
    struct ContributorHandle
    {
        /** @brief Nonzero contributor identifier, or zero for an invalid handle. */
        uint64_t id = 0;

        /** @brief Check whether this handle identifies a registered contributor. @return True for a valid handle. */
        explicit operator bool() const
        {
            return id != 0;
        }
    };

    TfAggregationManager();
    ~TfAggregationManager();

    TfAggregationManager(const TfAggregationManager&) = delete;
    TfAggregationManager& operator=(const TfAggregationManager&) = delete;

    /**
     * @brief Register a TF contributor and associate it with a shared topic publisher.
     *
     * @param[in] contributorName Name used to identify the contributor in diagnostics.
     * @param[in] factory ROS 2 factory used to create the publisher and message. Must not be null.
     * @param[in] nodeHandle Shared node handle used by the publisher. Must not be null.
     * @param[in] contextHandle ROS 2 context that distinguishes the topic group. May be null.
     * @param[in] topicName Topic on which to publish the aggregated transforms.
     * @param[in] staticPublisher Whether the topic carries static transforms.
     * @param[in] qos Quality-of-service profile for the shared publisher.
     * @param[in] publishWithoutVerification Whether the group may publish without subscriber verification.
     * @return A valid contributor handle on success, or an invalid handle on failure.
     */
    ContributorHandle registerContributor(const std::string& contributorName,
                                          isaacsim::ros2::core::Ros2Factory* factory,
                                          std::shared_ptr<isaacsim::ros2::core::Ros2NodeHandle> nodeHandle,
                                          isaacsim::ros2::core::Ros2ContextHandle* contextHandle,
                                          const std::string& topicName,
                                          bool staticPublisher,
                                          const isaacsim::ros2::core::Ros2QoSProfile& qos,
                                          bool publishWithoutVerification);

    /** @brief Unregister a contributor. @param[in] handle Contributor to remove. Invalid handles are ignored. */
    void unregisterContributor(ContributorHandle handle);

    /**
     * @brief Submit the current transforms for a contributor.
     * @param[in] handle Contributor supplying the transforms.
     * @param[in] timeStamp ROS timestamp in seconds for dynamic transforms.
     * @param[in] transforms Transforms to include in the contributor's topic group.
     * @return True if the handle and its topic group are registered; otherwise, false.
     */
    bool submit(ContributorHandle handle,
                double timeStamp,
                const std::vector<isaacsim::ros2::core::TfTransformStamped>& transforms);

    /** @brief Publish all pending dynamic and changed static topic groups. */
    void flushAll();

    /** @brief Remove every contributor and topic group and restore the initial manager state. */
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

TfAggregationManager* getTfAggregationManager();
TfAggregationManager* getEnabledTfAggregationManager();
void setTfAggregationManager(TfAggregationManager* manager);

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
