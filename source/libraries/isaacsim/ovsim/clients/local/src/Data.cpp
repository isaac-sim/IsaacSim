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

#include <isaacsim/foundation/ovsim/data/Data.hpp>
#include <isaacsim/ovsim/clients/local/data/Data.hpp>
#include <isaacsim/physics/ovsim/data/Data.hpp>
#include <ovsim/interfaces/details/Exception.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace ovsim
{
namespace clients
{
namespace local
{
namespace data
{

namespace foundation = isaacsim::foundation::ovsim::data;
namespace physics = isaacsim::physics::ovsim::data;


OutputValueType read(const PathType& paths, const std::string& attributeName, std::optional<double> timestamp)
{
    using namespace ::ovsim::interfaces::details;
    try
    {
        return physics::read(paths, attributeName, timestamp);
    }
    catch (const InitializationError&)
    {
    }
    catch (const AttributeError&)
    {
    }

    try
    {
        return foundation::read(paths, attributeName, timestamp);
    }
    catch (const InitializationError&)
    {
        throw std::invalid_argument("Attribute '" + attributeName + "' could not be read: no backend is initialized");
    }
    catch (const AttributeError& e)
    {
        throw std::invalid_argument(e.what());
    }
}

void write(const PathType& paths,
           const std::string& attributeName,
           const InputValueType& values,
           std::optional<double> timestamp)
{
    using namespace ::ovsim::interfaces::details;
    try
    {
        physics::write(paths, attributeName, values, timestamp);
        return;
    }
    catch (const InitializationError&)
    {
    }
    catch (const AttributeError&)
    {
    }

    try
    {
        foundation::write(paths, attributeName, values, timestamp);
        return;
    }
    catch (const InitializationError&)
    {
        throw std::invalid_argument("Attribute '" + attributeName + "' could not be written: no backend is initialized");
    }
    catch (const AttributeError& e)
    {
        throw std::invalid_argument(e.what());
    }
}

} // namespace data
} // namespace local
} // namespace clients
} // namespace ovsim
} // namespace isaacsim
