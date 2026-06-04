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

#include "Tf2Impl.h"

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @brief Creates a new ROS 2 transform buffer instance.
 * @details
 * Factory method implementation that instantiates a concrete Ros2BufferCoreImpl object.
 *
 * @return Shared pointer to the newly created buffer instance.
 */
std::shared_ptr<Ros2BufferCore> Tf2FactoryImpl::createBuffer()
{
    return std::make_shared<Ros2BufferCoreImpl>();
}

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim

/**
 * @brief Creates a factory instance for TF2 components.
 * @details
 * Global factory function that creates and returns a new Tf2FactoryImpl instance.
 * This function is exported and can be dynamically loaded by the plugin system.
 *
 * @return Pointer to a newly created TF2 factory instance.
 */
isaacsim::ros2::tf_viewer::Tf2Factory* createFactory()
{
    return new isaacsim::ros2::tf_viewer::Tf2FactoryImpl();
}
