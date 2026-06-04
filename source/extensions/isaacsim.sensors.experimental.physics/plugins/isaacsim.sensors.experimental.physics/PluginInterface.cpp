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


#include "ContactSensorImpl.h"
#include "EffortSensorImpl.h"
#include "ImuSensorImpl.h"
#include "JointStateSensorImpl.h"
#include "RaycastSensorImpl.h"

#include <carb/PluginUtils.h>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.sensors.experimental.physics.plugin", "C++ sensor backend",
                                                    "NVIDIA", carb::PluginHotReload::eEnabled, "dev" };

CARB_EXPORT void carbOnPluginStartup()
{
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

CARB_PLUGIN_IMPL(g_kPluginDesc,
                 isaacsim::sensors::experimental::physics::ImuSensorImpl,
                 isaacsim::sensors::experimental::physics::ContactSensorImpl,
                 isaacsim::sensors::experimental::physics::EffortSensorImpl,
                 isaacsim::sensors::experimental::physics::JointStateSensorImpl,
                 isaacsim::sensors::experimental::physics::RaycastSensorImpl)

void fillInterface(isaacsim::sensors::experimental::physics::ImuSensorImpl& iface)
{
    (void)iface;
}

void fillInterface(isaacsim::sensors::experimental::physics::ContactSensorImpl& iface)
{
    (void)iface;
}

void fillInterface(isaacsim::sensors::experimental::physics::EffortSensorImpl& iface)
{
    (void)iface;
}

void fillInterface(isaacsim::sensors::experimental::physics::JointStateSensorImpl& iface)
{
    (void)iface;
}

void fillInterface(isaacsim::sensors::experimental::physics::RaycastSensorImpl& iface)
{
    (void)iface;
}
