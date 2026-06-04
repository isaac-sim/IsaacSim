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
#include <carb/logging/Log.h>

#include <omni/fabric/RationalTime.h>

namespace isaacsim
{

namespace core
{
namespace nodes
{

/**
 * @class CoreNodes
 * @brief Minimal interface for core node functionality.
 * @details
 * This interface doesn't have any functions, but just implementing it and acquiring will load your plugin,
 * trigger calls of carbOnPluginStartup() and carbOnPluginShutdown() methods and allow you to use other
 * Carbonite plugins. That by itself can get you quite far and is useful as a basic building block for Kit
 * extensions. One can define their own interface with own python bindings when needed and abandon this one.
 */
struct CoreNodes
{
    CARB_PLUGIN_INTERFACE("isaacsim::core::nodes", 3, 0);
};
} // nodes
} // core
} // isaacsim
