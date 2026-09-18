// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "BindingsPhysics.hpp"
#include "NanobindHelpers.hpp"

#include <isaacsim/physics/manager/PhysicsBenchmark.hpp>

namespace nb = nanobind;

void isaacsim::physics::manager::details::bindPhysicsBenchmarks(nb::module_& module)
{
    using namespace isaacsim::physics::manager;
    using namespace isaacsim::physics::registration;

    module.def(
        "subscribe_profile_stats_events",
        [](ProfileStatisticsNotificationFunction callback)
        {
            const SubscriptionId id = subscribeProfileStatisticsEvents(std::move(callback));
            return PythonSubscription(
                id, [](SubscriptionId subscriptionId) { unsubscribeProfileStatisticsEvents(subscriptionId); });
        },
        nb::arg("callback"),
        "Subscribe to physics simulation profile stats events.\n\n"
        "Returns a Subscription handle that unsubscribes automatically when dropped.");
}
