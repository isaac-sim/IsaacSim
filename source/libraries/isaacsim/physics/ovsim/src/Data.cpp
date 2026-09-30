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

#include "details/Registry.hpp"

#include <isaacsim/physics/ovsim/data/Data.hpp>
#include <ovsim/interfaces/details/Exception.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace physics
{
namespace ovsim
{
namespace data
{

namespace array = isaacsim::common::array;

OutputValueType read(const PathType& paths, const std::string& attributeName, std::optional<double> timestamp)
{
    // TODO: Add time-sampled attribute support.
    (void)timestamp;
    return details::getAttributeValues(details::processPaths(paths), attributeName);
}

void write(const PathType& paths,
           const std::string& attributeName,
           const InputValueType& values,
           std::optional<double> timestamp)
{
    // TODO: Add time-sampled attribute support.
    (void)timestamp;
    details::setAttributeValues(details::processPaths(paths), attributeName, values);
}

} // namespace data
} // namespace ovsim
} // namespace physics
} // namespace isaacsim
