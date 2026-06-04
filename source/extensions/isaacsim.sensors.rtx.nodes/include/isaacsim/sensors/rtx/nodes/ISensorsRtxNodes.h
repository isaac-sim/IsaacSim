// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Interface.h>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{
namespace nodes
{

/**
 * @brief Interface for RTX sensor OmniGraphnodes.
 *
 * This interface provides the foundation for RTX sensor OmniGraph nodes in Isaac Sim.
 * Implementing this interface:
 * - Enables plugin loading and initialization
 * - Triggers carbOnPluginStartup() and carbOnPluginShutdown() lifecycle methods
 * - Registers OGN nodes (IsaacExtractRTXSensorPointCloud, etc.) with the OmniGraph system
 */

struct ISensorsRtxNodes
{
    CARB_PLUGIN_INTERFACE("isaacsim::sensors::rtx::nodes::ISensorsRtxNodes", 0, 1);
};
} // nodes
} // rtx
} // sensors
} // isaacsim
