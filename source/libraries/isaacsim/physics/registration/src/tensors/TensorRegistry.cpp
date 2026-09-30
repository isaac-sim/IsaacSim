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

#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>

#include <mutex>
#include <new>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

struct TensorRegistry::Implementation
{
    mutable std::mutex mutex;
    // (engine, entityName) -> factory
    std::unordered_map<std::string, std::unordered_map<std::string, EntityFactory>> entityFactories;
    // Declared engine names. An engine is a kind of simulator; the maps above are keyed by simulation name,
    // of which one engine may have several. Keeping them apart is what stops a live simulation from reading
    // as an engine, and an engine from implying a simulation exists.
    std::set<std::string> engines;
};

namespace
{
// Holds a value in static storage and never runs its destructor — the NoDestructor
// idiom (cf. absl::NoDestructor / Chromium base::NoDestructor). For a
// process-lifetime singleton whose destructor has no safe time to run, this
// constructs the value once in place (no heap allocation) and leaves it alive until
// the OS reclaims the process.
template <typename Type>
class NoDestructor
{
public:
    NoDestructor()
    {
        ::new (static_cast<void*>(&m_storage)) Type();
    }
    NoDestructor(const NoDestructor&) = delete;
    NoDestructor& operator=(const NoDestructor&) = delete;

    Type& operator*()
    {
        return *get();
    }
    const Type& operator*() const
    {
        return *get();
    }
    Type* operator->()
    {
        return get();
    }
    const Type* operator->() const
    {
        return get();
    }
    Type* get()
    {
        // std::launder: the object was placement-new'd into m_storage, so access
        // through a pointer cast from the storage is only well-defined in C++17
        // after laundering (matches absl::NoDestructor / Chromium base::NoDestructor).
        return std::launder(reinterpret_cast<Type*>(&m_storage));
    }
    const Type* get() const
    {
        return std::launder(reinterpret_cast<const Type*>(&m_storage));
    }

private:
    alignas(Type) unsigned char m_storage[sizeof(Type)];
};
} // namespace

TensorRegistry& TensorRegistry::getInstance()
{
    static TensorRegistry s_registry;
    return s_registry;
}

TensorRegistry::Implementation& TensorRegistry::_getImplementationStorage()
{
    // Process-lifetime singleton, intentionally never destroyed. The factory
    // maps hold std::function callbacks that capture Python objects
    // (nanobind::callable) from the language bindings, so this destructor has no
    // safe time to run: mid-process it would break live consumers, and at
    // process exit it runs after Py_Finalize(), where releasing those Python
    // references use-after-frees the finalized interpreter. NoDestructor
    // constructs the Implementation once, in place, and never destructs it.
    static NoDestructor<Implementation> s_implementation;
    return *s_implementation;
}

TensorRegistry::Implementation& TensorRegistry::_getImplementation()
{
    return _getImplementationStorage();
}

const TensorRegistry::Implementation& TensorRegistry::_getImplementation() const
{
    return _getImplementationStorage();
}

bool TensorRegistry::registerEntity(const std::string& engine, const std::string& entityName, EntityFactory factory)
{
    if (!factory)
    {
        throw std::invalid_argument("registerEntity: factory must not be null (engine='" + engine + "', entity='" +
                                    entityName + "')");
    }
    Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    auto& engineMap = implementation.entityFactories[engine];
    const bool isNew = engineMap.find(entityName) == engineMap.end();
    engineMap[entityName] = std::move(factory);
    return isNew;
}

bool TensorRegistry::unregisterEntity(const std::string& engine, const std::string& entityName)
{
    Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    auto engineIterator = implementation.entityFactories.find(engine);
    if (engineIterator == implementation.entityFactories.end())
    {
        return false;
    }
    auto factoryIterator = engineIterator->second.find(entityName);
    if (factoryIterator == engineIterator->second.end())
    {
        return false;
    }
    engineIterator->second.erase(factoryIterator);
    if (engineIterator->second.empty())
    {
        implementation.entityFactories.erase(engineIterator);
    }
    return true;
}

bool TensorRegistry::hasEntity(const std::string& engine, const std::string& entityName) const
{
    const Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    const auto engineIterator = implementation.entityFactories.find(engine);
    if (engineIterator == implementation.entityFactories.end())
    {
        return false;
    }
    return engineIterator->second.find(entityName) != engineIterator->second.end();
}

std::shared_ptr<IEntityView> TensorRegistry::createEntity(const std::string& engine,
                                                          const std::string& entityName,
                                                          const std::vector<std::string>& paths,
                                                          const std::optional<EntityOptions>& options) const
{
    EntityFactory factory;
    {
        const Implementation& implementation = _getImplementation();
        std::lock_guard<std::mutex> lockGuard(implementation.mutex);
        const auto engineIterator = implementation.entityFactories.find(engine);
        if (engineIterator == implementation.entityFactories.end())
        {
            throw std::out_of_range("createEntity: no factories registered for engine '" + engine + "'");
        }
        const auto factoryIterator = engineIterator->second.find(entityName);
        if (factoryIterator == engineIterator->second.end())
        {
            throw std::out_of_range("createEntity: entity '" + entityName + "' not registered for engine '" + engine +
                                    "'");
        }
        factory = factoryIterator->second;
    }
    return factory(paths, options);
}

bool TensorRegistry::registerEngine(const std::string& engine)
{
    Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    return implementation.engines.insert(engine).second;
}

std::vector<std::string> TensorRegistry::listEngines() const
{
    const Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    return std::vector<std::string>(implementation.engines.begin(), implementation.engines.end());
}

std::vector<std::string> TensorRegistry::listSimulations() const
{
    const Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    // Sorted so the order is stable across calls; the entity map alone is authoritative now that entity
    // factories are the only way to address a simulation.
    std::set<std::string> uniqueSimulationNames;
    for (const auto& simulationEntry : implementation.entityFactories)
    {
        uniqueSimulationNames.insert(simulationEntry.first);
    }
    return std::vector<std::string>(uniqueSimulationNames.begin(), uniqueSimulationNames.end());
}

std::vector<std::string> TensorRegistry::listEntities(const std::string& engine) const
{
    const Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    const auto engineIterator = implementation.entityFactories.find(engine);
    if (engineIterator == implementation.entityFactories.end())
    {
        return {};
    }
    std::vector<std::string> entityNames;
    entityNames.reserve(engineIterator->second.size());
    for (const auto& factoryEntry : engineIterator->second)
    {
        entityNames.push_back(factoryEntry.first);
    }
    return entityNames;
}

void TensorRegistry::clearForTesting()
{
    Implementation& implementation = _getImplementation();
    std::lock_guard<std::mutex> lockGuard(implementation.mutex);
    implementation.entityFactories.clear();
    implementation.engines.clear();
}

} // namespace tensors
} // namespace physics
} // namespace isaacsim
