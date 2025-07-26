// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "rosidl_runtime_c/string_functions.h"

#include <isaacsim/ros2/bridge/Ros2Macros.h>
#include <rcl/rcl.h>

#include <inttypes.h>

namespace isaacsim
{
namespace ros2
{
namespace bridge
{

Ros2MessageInterfaceImpl::Ros2MessageInterfaceImpl(std::string pkgName,
                                                   std::string msgSubfolder,
                                                   std::string msgName,
                                                   BackendMessageType messageType,
                                                   bool showLoadingError)
    : Ros2MessageInterface(pkgName, msgSubfolder, msgName, messageType, showLoadingError)
{
}

void Ros2MessageInterfaceImpl::writeRosTime(const int64_t nanoseconds, builtin_interfaces__msg__Time& time)
{
    constexpr rcl_time_point_value_t kRemainder = RCL_S_TO_NS(1);
    const auto result = std::div(nanoseconds, kRemainder);
    if (result.rem >= 0)
    {
        time.sec = static_cast<std::int32_t>(result.quot);
        time.nanosec = static_cast<std::uint32_t>(result.rem);
    }
    else
    {
        time.sec = static_cast<std::int32_t>(result.quot - 1);
        time.nanosec = static_cast<std::uint32_t>(kRemainder + result.rem);
    }
}

void Ros2MessageInterfaceImpl::writeRosString(const std::string& input, rosidl_runtime_c__String& output)
{
    rosidl_runtime_c__String__assign(&output, input.c_str());
}

void Ros2MessageInterfaceImpl::writeRosHeader(const std::string& frameId,
                                              const int64_t nanoseconds,
                                              std_msgs__msg__Header& header)
{
    writeRosString(frameId.c_str(), header.frame_id);
    if (nanoseconds > 0)
    {
        writeRosTime(nanoseconds, header.stamp);
    }
    else
    {
        fprintf(stdout,
                "[Warning] Frame %s Timestamp is invalid. Timestamp %" PRId64
                " will be neglected for all published ROS messages\n",
                frameId.c_str(), nanoseconds);
    }
}

} // namespace bridge
} // namespace ros2
} // namespace isaacsim
