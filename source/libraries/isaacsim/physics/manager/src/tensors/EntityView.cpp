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

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

namespace
{
// Dispatch only consumes supports/shape flags; unknown legacy dtypes stay in the raw metadata.
ArrayTensorSpecification arrayDispatchCapabilities(TensorSpecification specification)
{
    if (specification.dtype == DType::eUnknown)
        specification.dtype = DType::eFloat32;
    return toArraySpecification(specification);
}
}

struct EntityView::Implementation
{
    struct GetEntry
    {
        GetArrayImplementationFunction callback;
        ArrayTensorSpecification specification;
    };
    struct SetEntry
    {
        SetArrayImplementationFunction callback;
        ArrayTensorSpecification specification;
    };
    struct GetMultiEntry
    {
        GetArrayMultiImplementationFunction callback;
        ArrayTensorSpecification specification;
        std::vector<ArrayTensorSpecification> outputSpecifications;
    };
    struct SetMultiEntry
    {
        SetArrayMultiImplementationFunction callback;
        ArrayTensorSpecification specification;
    };

    std::unordered_map<std::string, GetEntry> getImplementations;
    std::unordered_map<std::string, SetEntry> setImplementations;
    std::unordered_map<std::string, GetMultiEntry> getMultiImplementations;
    std::unordered_map<std::string, SetMultiEntry> setMultiImplementations;
    std::unordered_map<std::string, MetadataImplementationFunction> metadataImplementations;
};

struct EntityView::LegacyImplementation
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

bool EntityView::registerArrayImplementation(const std::string& operationName,
                                             ImplementationKind kind,
                                             GetArrayImplementationFunction callback,
                                             ArrayTensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerArrayImplementation(GetArrayImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->getMultiImplementations.find(operationName) != m_implementation->getMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerArrayImplementation: GET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    const bool isNew =
        m_implementation->getImplementations.find(operationName) == m_implementation->getImplementations.end();
    m_implementation->getImplementations[operationName] =
        Implementation::GetEntry{ std::move(callback), std::move(specification) };
    if (m_legacy)
        m_legacy->getImplementations.erase(operationName);
    return isNew;
}

bool EntityView::registerArrayImplementation(const std::string& operationName,
                                             ImplementationKind kind,
                                             SetArrayImplementationFunction callback,
                                             ArrayTensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerArrayImplementation(SetArrayImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->setMultiImplementations.find(operationName) != m_implementation->setMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerArrayImplementation: SET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    const bool isNew =
        m_implementation->setImplementations.find(operationName) == m_implementation->setImplementations.end();
    m_implementation->setImplementations[operationName] =
        Implementation::SetEntry{ std::move(callback), std::move(specification) };
    if (m_legacy)
        m_legacy->setImplementations.erase(operationName);
    return isNew;
}

bool EntityView::registerArrayImplementation(const std::string& operationName,
                                             ImplementationKind kind,
                                             GetArrayMultiImplementationFunction callback,
                                             ArrayTensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerArrayImplementation(GetArrayMultiImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->getImplementations.find(operationName) != m_implementation->getImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerArrayImplementation: GET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    const bool isNew = m_implementation->getMultiImplementations.find(operationName) ==
                       m_implementation->getMultiImplementations.end();
    m_implementation->getMultiImplementations[operationName] =
        Implementation::GetMultiEntry{ std::move(callback), std::move(specification), {} };
    if (m_legacy)
        m_legacy->getMultiImplementations.erase(operationName);
    return isNew;
}

bool EntityView::registerArrayImplementation(const std::string& operationName,
                                             ImplementationKind kind,
                                             GetArrayMultiImplementationFunction callback,
                                             std::vector<ArrayTensorSpecification> outputSpecifications)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerArrayImplementation(GetArrayMultiImplementationFunction, outputSpecifications): kind must be "
            "ImplementationKind::eGet for implementation '" +
            operationName + "'");
    }
    if (m_implementation->getImplementations.find(operationName) != m_implementation->getImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerArrayImplementation: GET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    // The aggregate specification drives the supports and indexed-read gates in getArrayDataMulti. The per-output
    // specifications drive framework-side device allocation.
    ArrayTensorSpecification aggregateSpecification;
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
    if (m_legacy)
        m_legacy->getMultiImplementations.erase(operationName);
    return isNew;
}

bool EntityView::registerArrayImplementation(const std::string& operationName,
                                             ImplementationKind kind,
                                             SetArrayMultiImplementationFunction callback,
                                             ArrayTensorSpecification specification)
{
    if (!m_implementation)
    {
        m_implementation = std::make_unique<Implementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerArrayImplementation(SetArrayMultiImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_implementation->setImplementations.find(operationName) != m_implementation->setImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerArrayImplementation: SET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    const bool isNew = m_implementation->setMultiImplementations.find(operationName) ==
                       m_implementation->setMultiImplementations.end();
    m_implementation->setMultiImplementations[operationName] =
        Implementation::SetMultiEntry{ std::move(callback), std::move(specification) };
    if (m_legacy)
        m_legacy->setMultiImplementations.erase(operationName);
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

    const auto isSupported = [](const ArrayTensorSpecification& specification) { return specification.supports; };

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

ArrayTensorSpecification EntityView::getArrayImplementationSpecification(const std::string& operationName,
                                                                         ImplementationKind kind) const
{
    if (m_legacy)
    {
        if (kind == ImplementationKind::eGet)
        {
            const auto single = m_legacy->getImplementations.find(operationName);
            if (single != m_legacy->getImplementations.end())
                return toArraySpecification(single->second.specification);
            const auto multi = m_legacy->getMultiImplementations.find(operationName);
            if (multi != m_legacy->getMultiImplementations.end())
                return toArraySpecification(multi->second.specification);
        }
        else
        {
            const auto single = m_legacy->setImplementations.find(operationName);
            if (single != m_legacy->setImplementations.end())
                return toArraySpecification(single->second.specification);
            const auto multi = m_legacy->setMultiImplementations.find(operationName);
            if (multi != m_legacy->setMultiImplementations.end())
                return toArraySpecification(multi->second.specification);
        }
    }

    if (!m_implementation)
    {
        throw std::out_of_range("EntityView::getArrayImplementationSpecification: no implementations registered for '" +
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
    throw std::out_of_range("EntityView::getArrayImplementationSpecification: implementation '" + operationName +
                            "' not registered for the requested kind");
}

std::vector<ArrayTensorSpecification> EntityView::getArrayMultiImplementationSpecifications(
    const std::string& operationName, ImplementationKind kind) const
{
    if (m_legacy && kind == ImplementationKind::eGet)
    {
        const auto multi = m_legacy->getMultiImplementations.find(operationName);
        if (multi != m_legacy->getMultiImplementations.end())
        {
            std::vector<ArrayTensorSpecification> specifications;
            for (const auto& value : multi->second.outputSpecifications)
                specifications.push_back(toArraySpecification(value));
            return specifications;
        }
    }

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
        if (m_legacy)
        {
            const auto legacy = m_legacy->getImplementations.find(operationName);
            if (legacy != m_legacy->getImplementations.end())
                legacy->second.specification.shapeHint = shapeHint;
        }
        return true;
    }
    const auto implementationIterator = m_implementation->setImplementations.find(operationName);
    if (implementationIterator == m_implementation->setImplementations.end())
    {
        return false;
    }
    implementationIterator->second.specification.shapeHint = shapeHint;
    if (m_legacy)
    {
        const auto legacy = m_legacy->setImplementations.find(operationName);
        if (legacy != m_legacy->setImplementations.end())
            legacy->second.specification.shapeHint = shapeHint;
    }
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
    std::vector<ArrayTensorSpecification>& outputSpecifications =
        multiImplementationIterator->second.outputSpecifications;
    if (shapeHints.size() != outputSpecifications.size())
    {
        throw std::invalid_argument("EntityView::_setImplementationOutputShapeHints: implementation '" + operationName +
                                    "' has " + std::to_string(outputSpecifications.size()) + " outputs, received " +
                                    std::to_string(shapeHints.size()));
    }
    for (size_t outputIndex = 0; outputIndex < outputSpecifications.size(); ++outputIndex)
    {
        outputSpecifications[outputIndex].shapeHint = shapeHints[outputIndex];
        if (m_legacy)
        {
            const auto legacy = m_legacy->getMultiImplementations.find(operationName);
            if (legacy != m_legacy->getMultiImplementations.end())
                legacy->second.outputSpecifications[outputIndex].shapeHint = shapeHints[outputIndex];
        }
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

array::Array EntityView::getArrayData(const std::string& operationName,
                                      const std::optional<array::Array>& indices,
                                      const std::optional<array::Array>& output) const
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
    if (indices.has_value() && !implementationIterator->second.specification.supportsIndexedRead)
    {
        throw std::invalid_argument("EntityView::getData: implementation '" + operationName +
                                    "' does not support indexed reads");
    }
    return implementationIterator->second.callback(indices, output);
}

std::vector<array::Array> EntityView::getArrayDataMulti(const std::string& operationName,
                                                        const std::optional<array::Array>& indices,
                                                        const std::optional<std::vector<array::Array>>& output) const
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
    if (indices.has_value() && !implementationIterator->second.specification.supportsIndexedRead)
    {
        throw std::invalid_argument("EntityView::getDataMulti: implementation '" + operationName +
                                    "' does not support indexed reads");
    }
    return implementationIterator->second.callback(indices, output);
}

void EntityView::setArrayData(const std::string& operationName,
                              const array::Array& data,
                              const std::optional<array::Array>& indices) const
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
    if (indices.has_value() && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setData: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    implementationIterator->second.callback(data, indices);
}

void EntityView::setArrayDataMulti(const std::string& operationName,
                                   const std::vector<array::Array>& data,
                                   const std::optional<array::Array>& indices) const
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
    if (indices.has_value() && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setDataMulti: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    implementationIterator->second.callback(data, indices);
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetImplementationFunction callback,
                                        TensorSpecification specification)
{
    GetArrayImplementationFunction bridge =
        [callback](const std::optional<array::Array>& indices, const std::optional<array::Array>& output)
    { return toArray(callback(toTensorDescription(indices), toTensorDescription(output))).value(); };
    const bool registeredNew =
        registerArrayImplementation(operationName, kind, std::move(bridge), arrayDispatchCapabilities(specification));

    if (!m_legacy)
    {
        m_legacy = std::make_unique<LegacyImplementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_legacy->getMultiImplementations.find(operationName) != m_legacy->getMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: GET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    m_legacy->getImplementations[operationName] =
        LegacyImplementation::GetEntry{ std::move(callback), std::move(specification) };
    return registeredNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        SetImplementationFunction callback,
                                        TensorSpecification specification)
{
    SetArrayImplementationFunction bridge = [callback](
                                                const array::Array& data, const std::optional<array::Array>& indices)
    { callback(toTensorDescription(data), toTensorDescription(indices)); };
    const bool registeredNew =
        registerArrayImplementation(operationName, kind, std::move(bridge), arrayDispatchCapabilities(specification));

    if (!m_legacy)
    {
        m_legacy = std::make_unique<LegacyImplementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerImplementation(SetImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_legacy->setMultiImplementations.find(operationName) != m_legacy->setMultiImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: SET implementation '" + operationName +
                                    "' is already registered as multi-buffer");
    }
    m_legacy->setImplementations[operationName] =
        LegacyImplementation::SetEntry{ std::move(callback), std::move(specification) };
    return registeredNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetMultiImplementationFunction callback,
                                        TensorSpecification specification)
{
    GetArrayMultiImplementationFunction bridge =
        [callback](const std::optional<array::Array>& indices, const std::optional<std::vector<array::Array>>& output)
    {
        std::vector<TensorDescription> buffers;
        if (output)
            for (const auto& value : *output)
                buffers.push_back(toTensorDescription(value));
        const auto values = callback(toTensorDescription(indices), buffers);
        std::vector<array::Array> result;
        for (const auto& value : values)
            result.push_back(toArray(value).value());
        return result;
    };
    const bool registeredNew =
        registerArrayImplementation(operationName, kind, std::move(bridge), arrayDispatchCapabilities(specification));

    if (!m_legacy)
    {
        m_legacy = std::make_unique<LegacyImplementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetMultiImplementationFunction): kind must be ImplementationKind::eGet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_legacy->getImplementations.find(operationName) != m_legacy->getImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: GET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    m_legacy->getMultiImplementations[operationName] =
        LegacyImplementation::GetMultiEntry{ std::move(callback), std::move(specification), {} };
    return registeredNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        GetMultiImplementationFunction callback,
                                        std::vector<TensorSpecification> outputSpecifications)
{
    GetArrayMultiImplementationFunction bridge =
        [callback](const std::optional<array::Array>& indices, const std::optional<std::vector<array::Array>>& output)
    {
        std::vector<TensorDescription> buffers;
        if (output)
            for (const auto& value : *output)
                buffers.push_back(toTensorDescription(value));
        const auto values = callback(toTensorDescription(indices), buffers);
        std::vector<array::Array> result;
        for (const auto& value : values)
            result.push_back(toArray(value).value());
        return result;
    };
    std::vector<ArrayTensorSpecification> arraySpecifications;
    for (const auto& value : outputSpecifications)
        arraySpecifications.push_back(arrayDispatchCapabilities(value));
    const bool registeredNew =
        registerArrayImplementation(operationName, kind, std::move(bridge), std::move(arraySpecifications));

    if (!m_legacy)
    {
        m_legacy = std::make_unique<LegacyImplementation>();
    }
    if (kind != ImplementationKind::eGet)
    {
        throw std::invalid_argument(
            "registerImplementation(GetMultiImplementationFunction, outputSpecifications): kind must be "
            "ImplementationKind::eGet for implementation '" +
            operationName + "'");
    }
    if (m_legacy->getImplementations.find(operationName) != m_legacy->getImplementations.end())
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
    m_legacy->getMultiImplementations[operationName] =
        LegacyImplementation::GetMultiEntry{ std::move(callback), aggregateSpecification,
                                             std::move(outputSpecifications) };
    return registeredNew;
}

bool EntityView::registerImplementation(const std::string& operationName,
                                        ImplementationKind kind,
                                        SetMultiImplementationFunction callback,
                                        TensorSpecification specification)
{
    SetArrayMultiImplementationFunction bridge =
        [callback](const std::vector<array::Array>& data, const std::optional<array::Array>& indices)
    {
        std::vector<TensorDescription> buffers;
        for (const auto& value : data)
            buffers.push_back(toTensorDescription(value));
        callback(buffers, toTensorDescription(indices));
    };
    const bool registeredNew =
        registerArrayImplementation(operationName, kind, std::move(bridge), arrayDispatchCapabilities(specification));

    if (!m_legacy)
    {
        m_legacy = std::make_unique<LegacyImplementation>();
    }
    if (kind != ImplementationKind::eSet)
    {
        throw std::invalid_argument(
            "registerImplementation(SetMultiImplementationFunction): kind must be ImplementationKind::eSet for "
            "implementation '" +
            operationName + "'");
    }
    if (m_legacy->setImplementations.find(operationName) != m_legacy->setImplementations.end())
    {
        throw std::invalid_argument("EntityView::registerImplementation: SET implementation '" + operationName +
                                    "' is already registered as single-buffer");
    }
    m_legacy->setMultiImplementations[operationName] =
        LegacyImplementation::SetMultiEntry{ std::move(callback), std::move(specification) };
    return registeredNew;
}

TensorSpecification EntityView::getImplementationSpecification(const std::string& operationName,
                                                               ImplementationKind kind) const
{
    if (!m_legacy)
    {
        return toTensorSpecification(getArrayImplementationSpecification(operationName, kind));
    }

    if (!m_legacy)
    {
        throw std::out_of_range("EntityView::getImplementationSpecification: no implementations registered for '" +
                                operationName + "'");
    }

    if (kind == ImplementationKind::eGet)
    {
        const auto implementationIterator = m_legacy->getImplementations.find(operationName);
        if (implementationIterator != m_legacy->getImplementations.end())
        {
            return implementationIterator->second.specification;
        }
        const auto multiImplementationIterator = m_legacy->getMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_legacy->getMultiImplementations.end())
        {
            return multiImplementationIterator->second.specification;
        }
    }
    else
    {
        const auto implementationIterator = m_legacy->setImplementations.find(operationName);
        if (implementationIterator != m_legacy->setImplementations.end())
        {
            return implementationIterator->second.specification;
        }
        const auto multiImplementationIterator = m_legacy->setMultiImplementations.find(operationName);
        if (multiImplementationIterator != m_legacy->setMultiImplementations.end())
        {
            return multiImplementationIterator->second.specification;
        }
    }
    return toTensorSpecification(getArrayImplementationSpecification(operationName, kind));
}

std::vector<TensorSpecification> EntityView::getMultiImplementationSpecifications(const std::string& operationName,
                                                                                  ImplementationKind kind) const
{
    if (!m_legacy || m_legacy->getMultiImplementations.find(operationName) == m_legacy->getMultiImplementations.end())
    {
        std::vector<TensorSpecification> result;
        for (const auto& value : getArrayMultiImplementationSpecifications(operationName, kind))
            result.push_back(toTensorSpecification(value));
        return result;
    }

    if (!m_legacy || kind != ImplementationKind::eGet)
    {
        return {};
    }
    const auto multiImplementationIterator = m_legacy->getMultiImplementations.find(operationName);
    if (multiImplementationIterator != m_legacy->getMultiImplementations.end())
    {
        return multiImplementationIterator->second.outputSpecifications;
    }
    return {};
}

TensorDescription EntityView::getData(const std::string& operationName,
                                      const TensorDescription& indices,
                                      const TensorDescription& output) const
{
    if (m_legacy)
    {
        const auto legacy = m_legacy->getImplementations.find(operationName);
        if (legacy != m_legacy->getImplementations.end())
        {
            if (!legacy->second.specification.supports)
                throw std::runtime_error(
                    "EntityView::getData: implementation '" + operationName +
                    "' is registered but reports supports=false (operation not implemented for this engine)");
            if (!indices.isEmpty() && !legacy->second.specification.supportsIndexedRead)
                throw std::invalid_argument("EntityView::getData: implementation '" + operationName +
                                            "' does not support indexed reads");
            return legacy->second.callback(indices, output);
        }
    }
    const auto implementationIterator = m_implementation->getImplementations.find(operationName);
    if (implementationIterator == m_implementation->getImplementations.end())
        throw std::out_of_range("EntityView::getData: get-implementation '" + operationName + "' not registered");
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::getData: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    const auto arrayIndices = toArray(indices);
    if (arrayIndices && !implementationIterator->second.specification.supportsIndexedRead)
    {
        throw std::invalid_argument("EntityView::getData: implementation '" + operationName +
                                    "' does not support indexed reads");
    }
    return toTensorDescription(implementationIterator->second.callback(arrayIndices, toArray(output)));
}

std::vector<TensorDescription> EntityView::getDataMulti(const std::string& operationName,
                                                        const TensorDescription& indices,
                                                        const std::vector<TensorDescription>& output) const
{
    if (m_legacy)
    {
        const auto legacy = m_legacy->getMultiImplementations.find(operationName);
        if (legacy != m_legacy->getMultiImplementations.end())
        {
            if (!legacy->second.specification.supports)
                throw std::runtime_error(
                    "EntityView::getDataMulti: implementation '" + operationName +
                    "' is registered but reports supports=false (operation not implemented for this engine)");
            if (!indices.isEmpty() && !legacy->second.specification.supportsIndexedRead)
                throw std::invalid_argument("EntityView::getDataMulti: implementation '" + operationName +
                                            "' does not support indexed reads");
            return legacy->second.callback(indices, output);
        }
    }
    const auto implementationIterator = m_implementation->getMultiImplementations.find(operationName);
    if (implementationIterator == m_implementation->getMultiImplementations.end())
        throw std::out_of_range("EntityView::getDataMulti: multi-get implementation '" + operationName +
                                "' not registered");
    if (!implementationIterator->second.specification.supports)
        throw std::runtime_error("EntityView::getDataMulti: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    const auto arrayIndices = toArray(indices);
    if (arrayIndices && !implementationIterator->second.specification.supportsIndexedRead)
        throw std::invalid_argument("EntityView::getDataMulti: implementation '" + operationName +
                                    "' does not support indexed reads");

    {
        // Array providers accept complete destination lists only. For legacy sparse lists,
        // use their allocation route, then populate the destinations that the caller supplied.
        if (std::any_of(output.begin(), output.end(),
                        [](const auto& value) { return value.data == nullptr && value.shape.empty(); }))
        {
            std::vector<std::optional<array::Array>> destinations;
            destinations.reserve(output.size());
            for (const auto& value : output)
                destinations.push_back(toArray(value));
            const auto values = implementationIterator->second.callback(arrayIndices, std::nullopt);
            if (output.size() > values.size())
                throw std::invalid_argument("EntityView::getDataMulti: sparse output count exceeds result count");
            destinations.resize(values.size());
            // Validate every destination before writing any caller memory. Device differences
            // are supported by Array::set, but implicit dtype conversion/broadcasting is not.
            for (size_t index = 0; index < values.size(); ++index)
            {
                if (!destinations[index])
                    continue;
                const auto& destination = *destinations[index];
                if (destination.shape() != values[index].shape() || destination.dtype() != values[index].dtype())
                    throw std::invalid_argument("EntityView::getDataMulti: sparse output shape or dtype mismatch");
                if (!destination.isWriteable())
                    throw std::invalid_argument("EntityView::getDataMulti: sparse output layout is not writeable");
            }
            // Provider results can alias any caller destination, including another slot.
            // Snapshot all results before writing so cross-slot aliases cannot corrupt reads
            // or an omitted result. This cost is confined to the sparse compatibility path.
            const bool hasDestination = std::any_of(
                destinations.begin(), destinations.end(), [](const auto& value) { return value.has_value(); });
            std::vector<array::Array> snapshots;
            snapshots.reserve(values.size());
            for (const auto& value : values)
                snapshots.push_back(hasDestination ? value.copy() : value);
            std::vector<TensorDescription> result;
            result.reserve(values.size());
            for (size_t index = 0; index < values.size(); ++index)
            {
                if (destinations[index])
                {
                    destinations[index]->set(snapshots[index]);
                    result.push_back(output[index]);
                }
                else
                    result.push_back(toTensorDescription(snapshots[index]));
            }
            return result;
        }
        std::optional<std::vector<array::Array>> buffers;
        if (!output.empty())
        {
            buffers.emplace();
            for (const auto& value : output)
                buffers->push_back(toArray(value).value());
        }
        std::vector<TensorDescription> result;
        for (const auto& value : implementationIterator->second.callback(arrayIndices, buffers))
            result.push_back(toTensorDescription(value));
        return result;
    }
}

void EntityView::setData(const std::string& operationName,
                         const TensorDescription& data,
                         const TensorDescription& indices) const
{
    if (m_legacy)
    {
        const auto legacy = m_legacy->setImplementations.find(operationName);
        if (legacy != m_legacy->setImplementations.end())
        {
            if (!legacy->second.specification.supports)
                throw std::runtime_error(
                    "EntityView::setData: implementation '" + operationName +
                    "' is registered but reports supports=false (operation not implemented for this engine)");
            if (!indices.isEmpty() && !legacy->second.specification.supportsIndexedWrite)
                throw std::invalid_argument("EntityView::setData: implementation '" + operationName +
                                            "' does not support indexed writes");
            legacy->second.callback(data, indices);
            return;
        }
    }
    const auto implementationIterator = m_implementation->setImplementations.find(operationName);
    if (implementationIterator == m_implementation->setImplementations.end())
        throw std::out_of_range("EntityView::setData: set-implementation '" + operationName + "' not registered");
    if (!implementationIterator->second.specification.supports)
    {
        throw std::runtime_error("EntityView::setData: implementation '" + operationName +
                                 "' is registered but reports supports=false (operation not implemented for this engine)");
    }
    const auto arrayIndices = toArray(indices);
    if (arrayIndices && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setData: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    const auto arrayData = toArray(data);
    implementationIterator->second.callback(
        arrayData ?
            *arrayData :
            array::Array::fromBuffer(nullptr, array::Shape{ 0 }, implementationIterator->second.specification.dtype),
        arrayIndices);
}

void EntityView::setDataMulti(const std::string& operationName,
                              const std::vector<TensorDescription>& data,
                              const TensorDescription& indices) const
{
    if (m_legacy)
    {
        const auto legacy = m_legacy->setMultiImplementations.find(operationName);
        if (legacy != m_legacy->setMultiImplementations.end())
        {
            if (!legacy->second.specification.supports)
                throw std::runtime_error(
                    "EntityView::setDataMulti: implementation '" + operationName +
                    "' is registered but reports supports=false (operation not implemented for this engine)");
            if (!indices.isEmpty() && !legacy->second.specification.supportsIndexedWrite)
                throw std::invalid_argument("EntityView::setDataMulti: implementation '" + operationName +
                                            "' does not support indexed writes");
            legacy->second.callback(data, indices);
            return;
        }
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
    const auto arrayIndices = toArray(indices);
    if (arrayIndices && !implementationIterator->second.specification.supportsIndexedWrite)
    {
        throw std::invalid_argument("EntityView::setDataMulti: implementation '" + operationName +
                                    "' does not support indexed writes");
    }
    std::vector<array::Array> buffers;
    buffers.reserve(data.size());
    for (const auto& value : data)
    {
        const auto arrayValue = toArray(value);
        buffers.push_back(arrayValue ? *arrayValue :
                                       array::Array::fromBuffer(nullptr, array::Shape{ 0 },
                                                                implementationIterator->second.specification.dtype));
    }
    implementationIterator->second.callback(buffers, arrayIndices);
}

} // namespace tensors
} // namespace physics
} // namespace isaacsim
