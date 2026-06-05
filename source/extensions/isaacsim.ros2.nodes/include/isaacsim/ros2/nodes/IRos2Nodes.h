// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
namespace ros2
{
namespace nodes
{

/**
 * @brief Interface for ROS2 nodes functionality.
 *
 * This interface provides the foundation for ROS2 nodes in Isaac Sim.
 * While it contains no direct functions, implementing this interface:
 * - Enables plugin loading and initialization
 * - Triggers carbOnPluginStartup() and carbOnPluginShutdown() lifecycle methods
 * - Provides access to the Carbonite plugin ecosystem
 *
 * Extensions can build upon this interface by defining custom functionality
 * and Python bindings as needed.
 */
struct IRos2Nodes
{
    CARB_PLUGIN_INTERFACE("isaacsim::ros2::nodes", 1, 0);
};

} // namespace nodes
} // namespace ros2
} // namespace isaacsim
