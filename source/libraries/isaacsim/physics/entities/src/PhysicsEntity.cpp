// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "details/EntityUtils.hpp"

#include <isaacsim/physics/entities/PhysicsEntity.hpp>
#include <isaacsim/physics/manager/PhysicsManager.hpp>

namespace isaacsim
{
namespace physics
{
namespace entities
{

PhysicsEntity::PhysicsEntity(const std::string& engine,
                             const std::string& entity,
                             const std::variant<std::string, std::vector<std::string>>& paths)
{
    m_entityView = std::dynamic_pointer_cast<tensors::EntityView>(
        manager::PhysicsManager::getInstance().createEntity(engine, entity, paths));
    if (!m_entityView)
    {
        throw std::runtime_error("Failed to create EntityView for `" + entity + "`");
    }
}

size_t PhysicsEntity::numPrims() const noexcept
{
    return m_entityView->getEntityCount();
}

array::Array PhysicsEntity::getData(const std::string& name, const std::optional<array::Array>& indices)
{
    return m_entityView->getArrayData(name, indices);
}

void PhysicsEntity::setData(const std::string& name, const array::Array& data, const std::optional<array::Array>& indices)
{
    m_entityView->setArrayData(name, data, indices);
}

std::vector<array::Array> PhysicsEntity::getMultiData(const std::string& name, const std::optional<array::Array>& indices)
{
    return m_entityView->getArrayDataMulti(name, indices);
}

void PhysicsEntity::setMultiData(const std::string& name,
                                 const std::vector<array::Array>& data,
                                 const std::optional<array::Array>& indices)
{
    m_entityView->setArrayDataMulti(name, data, indices);
}

} // namespace entities
} // namespace physics
} // namespace isaacsim
