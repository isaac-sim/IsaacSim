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

#include <isaacsim/physics/manager/tensors/EntityView.hpp>

#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

struct EntityView::Implementation
{
    struct GetEntry
    {
        GetImplementationFunction callback;
        TensorSpecification specification;
    };
    struct SetEntry
    {
        SetImplementationFunction callback;
        TensorSpecification specification;
    };
    struct GetMultiEntry
    {
        GetMultiImplementationFunction callback;
        TensorSpecification specification;
        std::vector<TensorSpecification> outputSpecifications;
    };
    struct SetMultiEntry
    {
        SetMultiImplementationFunction callback;
        TensorSpecification specification;
    };

    std::unordered_map<std::string, GetEntry> getImplementations;
    std::unordered_map<std::string, SetEntry> setImplementations;
    std::unordered_map<std::string, GetMultiEntry> getMultiImplementations;
    std::unordered_map<std::string, SetMultiEntry> setMultiImplementations;
    std::unordered_map<std::string, MetadataImplementationFunction> metadataImplementations;
};

EntityView::EntityView() : m_implementation(std::make_unique<Implementation>())
{
}

EntityView::EntityView(std::vector<std::string> primPathPatterns)
    : m_primPathPatterns(std::move(primPathPatterns)), m_implementation(std::make_unique<Implementation>())
{
}

EntityView::~EntityView() = default;

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetImplementationFunction callback,
                                        TensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->getMultiImplementations.find(operationName) != m_implementation->getMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: GET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    const bool isNew =
        m_implementation->getImplementations.find(operationName) == m_implementation->getImplementations.end();
    m_implementation->getImplementations[operationName] =
        Implementation::GetEntry{ std::move(callback), std::move(specification) };
    return isNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        SetImplementationFunction callback,
                                        TensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerImplementation(SetImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->setMultiImplementations.find(operationName) != m_implementation->setMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: SET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    const bool isNew =
        m_implementation->setImplementations.find(operationName) == m_implementation->setImplementations.end();
    m_implementation->setImplementations[operationName] =
        Implementation::SetEntry{ std::move(callback), std::move(specification) };
    return isNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetMultiImplementationFunction callback,
                                        TensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetMultiImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->getImplementations.find(operationName) != m_implementation->getImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: GET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    const bool isNew = m_implementation->getMultiImplementations.find(operationName) ==
                       m_implementation->getMultiImplementations.end();
    m_implementation->getMultiImplementations[operationName] =
        Implementation::GetMultiEntry{ std::move(callback), std::move(specification), {} };
    return isNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetMultiImplementationFunction callback,
                                        std::vector<TensorSpecification> outputSpecifications)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetMultiImplementationFunction, outputSpecifications): kind must be "
            "ImplementationKind::eGet for implementation '" +
            operationName + "'");
    }
    if (m_implementation->getImplementations.find(operationName) != m_implementation->getImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: GET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    // The aggregate specification drives the supports and indexed-read gates in getDataMulti. The per-output
    // specifications drive framework-side device allocation.
    TensorSpecification aggregateSpecification;
    aggregateSpecification.supports = !outputSpecifications.empty() && outputSpecifications.front().supports;
    if (!outputSpecifications.empty())
    {
        aggregateSpecification.dtype = outputSpecifications.front().dtype;
        aggregateSpecification.shapeHint = outputSpecifications.front().shapeHint;
        aggregateSpecification.supportsIndexedRead = outputSpecifications.front().supportsIndexedRead;
        aggregateSpecification.requiresHostData = outputSpecifications.front().requiresHostData;
    }
    const bool isNew = m_implementation->getMultiImplementations.find(operationName) ==
                       m_implementation->getMultiImplementations.end();
    m_implementation->getMultiImplementations[operationName] =
        Implementation::GetMultiEntry{ std::move(callback), aggregateSpecification, std::move(outputSpecifications) };
    return isNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        SetMultiImplementationFunction callback,
                                        TensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerImplementation(SetMultiImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->setImplementations.find(operationName) != m_implementation->setImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: SET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    const bool isNew = m_implementation->setMultiImplementations.find(operationName) ==
                       m_implementation->setMultiImplementations.end();
    m_implementation->setMultiImplementations[operationName] =
        Implementation::SetMultiEntry{ std::move(callback), std::move(specification) };
    return isNew;
}

bool EntityView::registerMetadata(const std::string& operationName, MetadataImplementationFunction callback)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    const bool isNew = m_implementation->metadataImplementations.find(operationName) ==
                       m_implementation->metadataImplementations.end();
    m_implementation->metadataImplementations[operationName] = std::move(callback);
    return isNew;
}

std::vector<std::string> EntityView::listImplementations(ImplementationKind kind) const
{
    std::vector<std::string> implementationNames;
    if (!m_implementation)
    {
        return implementationNames;
    }
    if (kind == ImplementationKind::eGet)
    {
        implementationNames.reserve(m_implementation->getImplementations.size() +
                                    m_implementation->getMultiImplementations.size());
        for (const auto& entry : m_implementation->getImplementations)
        {
            implementationNames.push_back(entry.first);
        }
        for (const auto& entry : m_implementation->getMultiImplementations)
        {
            implementationNames.push_back(entry.first);
        }
    }
    else
    {
        implementationNames.reserve(m_implementation->setImplementations.size() +
                                    m_implementation->setMultiImplementations.size());
        for (const auto& entry : m_implementation->setImplementations)
        {
            implementationNames.push_back(entry.first);
        }
        for (const auto& entry : m_implementation->setMultiImplementations)
        {
            implementationNames.push_back(entry.first);
        }
    }
    return implementationNames;
}

bool EntityView::hasImplementation(const std::string& operationName, ImplementationKind kind) const
{
    if (!m_implementation)
    {
        return false;
    }

    const auto isSupported = [](const TensorSpecification& specification) { return specification.supports; };

    if (kind == ImplementationKind::eGet)
    {
        const auto implementationIterator = m_implementation->getImplementations.find(operationName);
        if (implementationIterator != m_implementation->getImplementations.end())
        {
            return isSupported(implementationIterator->second.specification);
        }
        const auto multiImplementationIterator = m_implementation->getMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_implementation->getMultiImplementations.end())
        {
            return isSupported(multiImplementationIterator->second.specification);
        }
        return false;
    }
    else
    {
        const auto implementationIterator = m_implementation->setImplementations.find(operationName);
        if (implementationIterator != m_implementation->setImplementations.end())
        {
            return isSupported(implementationIterator->second.specification);
        }
        const auto multiImplementationIterator = m_implementation->setMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_implementation->setMultiImplementations.end())
        {
            return isSupported(multiImplementationIterator->second.specification);
        }
        return false;
    }
}

TensorSpecification EntityView::getImplementationSpecification(const std::string& operationName,
                                                               ImplementationKind kind) const
{
    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::getImplementationSpecification: no implementations registered for '" +
                                operationName + "'");
    }

    if (kind == ImplementationKind::eGet)
    {
        const auto implementationIterator = m_implementation->getImplementations.find(operationName);
        if (implementationIterator != m_implementation->getImplementations.end())
        {
            return implementationIterator->second.specification;
        }
        const auto multiImplementationIterator = m_implementation->getMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_implementation->getMultiImplementations.end())
        {
            return multiImplementationIterator->second.specification;
        }
    }
    else
    {
        const auto implementationIterator = m_implementation->setImplementations.find(operationName);
        if (implementationIterator != m_implementation->setImplementations.end())
        {
            return implementationIterator->second.specification;
        }
        const auto multiImplementationIterator = m_implementation->setMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_implementation->setMultiImplementations.end())
        {
            return multiImplementationIterator->second.specification;
        }
    }
    throw std::out_of_range("EntityView::getImplementationSpecification: implementation '" + operationName +
                            "' not registered for the requested kind");
}

std::vector<TensorSpecification> EntityView::getMultiImplementationSpecifications(const std::string& operationName,
                                                                                  ImplementationKind kind) const
{
    if (!m_implementation || kind != ImplementationKind::eGet)
    {
        return {};
    }
    const auto multiImplementationIterator = m_implementation->getMultiImplementations.find(operationName);
    if (multiImplementationIterator != m_implementation->getMultiImplementations.end())
    {
        return multiImplementationIterator->second.outputSpecifications;
    }
    return {};
}

bool EntityView::_setImplementationShapeHint(const std::string& operationName,
                                             ImplementationKind kind,
                                             const std::vector<int64_t>& shapeHint)
{
    if (!m_implementation)
    {
        return false;
    }
    if (kind == ImplementationKind::eGet)
    {
        const auto implementationIterator = m_implementation->getImplementations.find(operationName);
        if (implementationIterator == m_implementation->getImplementations.end())
        {
            return false;
        }
        implementationIterator->second.specification.shapeHint = shapeHint;
        return true;
    }
    const auto implementationIterator = m_implementation->setImplementations.find(operationName);
    if (implementationIterator == m_implementation->setImplementations.end())
    {
        return false;
    }
    implementationIterator->second.specification.shapeHint = shapeHint;
    return true;
}

bool EntityView::_setImplementationOutputShapeHints(const std::string& operationName,
                                                    ImplementationKind kind,
                                                    const std::vector<std::vector<int64_t>>& shapeHints)
{
    if (!m_implementation || kind != ImplementationKind::eGet)
    {
        return false;
    }
    const auto multiImplementationIterator = m_implementation->getMultiImplementations.find(operationName);
    if (multiImplementationIterator == m_implementation->getMultiImplementations.end())
    {
        return false;
    }
    std::vector<TensorSpecification>& outputSpecifications = multiImplementationIterator->second.outputSpecifications;
    if (shapeHints.size() != outputSpecifications.size())
    {
        throw std::invalid_argument("EntityView::_setImplementationOutputShapeHints: implementation '" + operationName +
                                    "' has " + std::to_string(outputSpecifications.size()) + " outputs, received " +
                                    std::to_string(shapeHints.size()));
    }
    for (size_t outputIndex = 0; outputIndex < outputSpecifications.size(); ++outputIndex)
    {
        outputSpecifications[outputIndex].shapeHint = shapeHints[outputIndex];
    }
    return true;
}

Metadata EntityView::getMetadata(const std::string& operationName) const
{
    if (!m_implementation)
    {
        return Metadata{};
    }
    const auto implementationIterator = m_implementation->metadataImplementations.find(operationName);
    if (implementationIterator == m_implementation->metadataImplementations.end())
    {
        return Metadata{};
    }
    return implementationIterator->second();
}

TensorDescription EntityView::getData(const std::string& operationName,
                                      const TensorDescription& indices,
                                      const TensorDescription& output) const
{
    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::getData: no implementations registered for '" + operationName + "'");
    }
    const auto implementationIterator = m_implementation->getImplementations.find(operationName);
    if (implementationIterator == m_implementation->getImplementations.end())
    {
        throw std::out_of_range("EntityView::getData: get-implementation '" + operationName + "' not registered");
    }
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::getData: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    if (!indices.isEmpty() && !implementationIterator->second.specification.supportsIndexedRead)
    {
        throw std::invalid_argument("EntityView::getData: implementation '" + operationName +
                                    "' does not support indexed reads");
    }
    return implementationIterator->second.callback(indices, output);
}

std::vector<TensorDescription> EntityView::getDataMulti(const std::string& operationName,
                                                        const TensorDescription& indices,
                                                        const std::vector<TensorDescription>& output) const
{
    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::getDataMulti: no implementations registered for '" + operationName + "'");
    }
    const auto implementationIterator = m_implementation->getMultiImplementations.find(operationName);
    if (implementationIterator == m_implementation->getMultiImplementations.end())
    {
        throw std::out_of_range("EntityView::getDataMulti: multi-get implementation '" + operationName +
                                "' not registered");
    }
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::getDataMulti: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    if (!indices.isEmpty() && !implementationIterator->second.specification.supportsIndexedRead)
    {
        throw std::invalid_argument("EntityView::getDataMulti: implementation '" + operationName +
                                    "' does not support indexed reads");
    }
    return implementationIterator->second.callback(indices, output);
}

void EntityView::setData(const std::string& operationName,
                         const TensorDescription& data,
                         const TensorDescription& indices) const
{
    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::setData: no implementations registered for '" + operationName + "'");
    }
    const auto implementationIterator = m_implementation->setImplementations.find(operationName);
    if (implementationIterator == m_implementation->setImplementations.end())
    {
        throw std::out_of_range("EntityView::setData: set-implementation '" + operationName + "' not registered");
    }
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::setData: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    if (!indices.isEmpty() && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setData: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    implementationIterator->second.callback(data, indices);
}

void EntityView::setDataMulti(const std::string& operationName,
                              const std::vector<TensorDescription>& data,
                              const TensorDescription& indices) const
{
    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::setDataMulti: no implementations registered for '" + operationName + "'");
    }
    const auto implementationIterator = m_implementation->setMultiImplementations.find(operationName);
    if (implementationIterator == m_implementation->setMultiImplementations.end())
    {
        throw std::out_of_range("EntityView::setDataMulti: multi-set implementation '" + operationName +
                                "' not registered");
    }
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::setDataMulti: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    if (!indices.isEmpty() && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setDataMulti: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    implementationIterator->second.callback(data, indices);
}

} // namespace tensors
} // namespace physics
} // namespace isaacsim
