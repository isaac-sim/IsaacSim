// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Ros2Impl.h"

#include <isaacsim/ros2/core/Ros2Macros.h>
#include <rcl/rcl.h>

namespace isaacsim
{
namespace ros2
{
namespace core
{

const std::map<Ros2QoSHistoryPolicy, rmw_qos_history_policy_t> g_kRos2QoSHistoryMap = {
    { Ros2QoSHistoryPolicy::eSystemDefault, RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT },
    { Ros2QoSHistoryPolicy::eKeepLast, RMW_QOS_POLICY_HISTORY_KEEP_LAST },
    { Ros2QoSHistoryPolicy::eKeepAll, RMW_QOS_POLICY_HISTORY_KEEP_ALL },
    { Ros2QoSHistoryPolicy::eUnknown, RMW_QOS_POLICY_HISTORY_UNKNOWN }
};


const std::map<Ros2QoSReliabilityPolicy, rmw_qos_reliability_policy_t> g_kRos2QoSReliabilityMap = {
    { Ros2QoSReliabilityPolicy::eSystemDefault, RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT },
    { Ros2QoSReliabilityPolicy::eReliable, RMW_QOS_POLICY_RELIABILITY_RELIABLE },
    { Ros2QoSReliabilityPolicy::eBestEffort, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT },
    { Ros2QoSReliabilityPolicy::eUnknown, RMW_QOS_POLICY_RELIABILITY_UNKNOWN }
};


const std::map<Ros2QoSDurabilityPolicy, rmw_qos_durability_policy_t> g_kRos2QoSDurabilityMap = {
    { Ros2QoSDurabilityPolicy::eSystemDefault, RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT },
    { Ros2QoSDurabilityPolicy::eTransientLocal, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL },
    { Ros2QoSDurabilityPolicy::eVolatile, RMW_QOS_POLICY_DURABILITY_VOLATILE },
    { Ros2QoSDurabilityPolicy::eUnknown, RMW_QOS_POLICY_DURABILITY_UNKNOWN }
};

// NOTE : RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE is deprecated and throws compiler errors,
//        so we handle this by just using the system default
const std::map<Ros2QoSLivelinessPolicy, rmw_qos_liveliness_policy_t> g_kRos2QoSLivelinessMap = {
    { Ros2QoSLivelinessPolicy::eSystemDefault, RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT },
    { Ros2QoSLivelinessPolicy::eAutomatic, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC },
    { Ros2QoSLivelinessPolicy::eManualByNode, RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT },
    { Ros2QoSLivelinessPolicy::eManualByTopic, RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC },
    { Ros2QoSLivelinessPolicy::eUnknown, RMW_QOS_POLICY_LIVELINESS_UNKNOWN }
};

rmw_time_t convertRos2QosTimeToImpl(const Ros2QoSTime& ros2Time)
{
    return { ros2Time.sec, ros2Time.nsec };
}

rmw_qos_profile_t Ros2QoSProfileConverter::convert(const Ros2QoSProfile& qos)
{
    rmw_qos_profile_t profile;

    profile.history = g_kRos2QoSHistoryMap.at(qos.history);
    profile.depth = qos.depth;
    profile.reliability = g_kRos2QoSReliabilityMap.at(qos.reliability);
    profile.durability = g_kRos2QoSDurabilityMap.at(qos.durability);
    profile.deadline = convertRos2QosTimeToImpl(qos.deadline);
    profile.lifespan = convertRos2QosTimeToImpl(qos.lifespan);
    profile.liveliness = g_kRos2QoSLivelinessMap.at(qos.liveliness);
    profile.liveliness_lease_duration = convertRos2QosTimeToImpl(qos.livelinessLeaseDuration);
    profile.avoid_ros_namespace_conventions = qos.avoidRosNamespaceConventions;

    return profile;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim
