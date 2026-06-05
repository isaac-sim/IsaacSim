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

#pragma once

#include <isaacsim/ros2/tf_viewer/Tf2Factory.h>

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @class Tf2FactoryImpl
 * @brief Factory implementation for creating TF2 components.
 * @details
 * Provides factory methods for creating ROS 2 transform buffer instances
 * and other TF2-related components.
 */
class Tf2FactoryImpl : public Tf2Factory
{
public:
    /**
     * @brief Creates a new ROS 2 transform buffer instance.
     * @return Shared pointer to the created buffer instance.
     */
    virtual std::shared_ptr<Ros2BufferCore> createBuffer();
};

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim

/// @cond DOXYGEN_SHOULD_SKIP_THIS
#ifdef _MSC_VER
extern "C" __declspec(dllexport) isaacsim::ros2::tf_viewer::Tf2Factory* createFactory();
#else
extern "C" isaacsim::ros2::tf_viewer::Tf2Factory* createFactory();
#endif
/// @endcond
