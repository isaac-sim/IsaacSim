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

#pragma once

#include "IEntityView.hpp"
#include "Metadata.hpp"

#include <isaacsim/physics/registration/Export.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

/**
 * @brief Engine-defined construction arguments for an entity view, keyed by option name.
 *
 * Paths alone cannot describe every entity view: a rigid-contact view also needs its contact filters and record
 * capacity, and a signed-distance-field view its query-point count. Each engine documents the option names it reads
 * and ignores the rest, so an option an engine does not recognize is not an error.
 */
using EntityOptions = std::unordered_map<std::string, Metadata>;

/**
 * @brief Factory that creates an engine-specific entity view for a set of USD paths.
 *
 * @param[in] paths The USD paths or path patterns represented by the new entity view. The reference remains valid only
 *                  for the duration of the factory invocation.
 * @param[in] options Engine-defined construction arguments, or @c std::nullopt to use the engine's defaults. The
 *                    reference remains valid only for the duration of the factory invocation.
 * @return Shared ownership of the created entity view, or @c nullptr when the factory does not create a view.
 */
using EntityFactory = std::function<std::shared_ptr<IEntityView>(
    const std::vector<std::string>& paths, const std::optional<EntityOptions>& options)>;

/**
 * @brief Process-wide registry of engine-specific tensor-view factories.
 *
 * Entity factories are keyed by engine and entity name. Registration, removal, lookup, and enumeration are safe to
 * call concurrently.
 *
 * Replacing, unregistering, or clearing a factory does not cancel or wait for a create operation that is already
 * invoking it. An in-flight invocation may retain the factory and its captured resources until the invocation
 * finishes.
 */
class ISAACSIM_PHYSICS_REGISTRATION_API TensorRegistry
{
public:
    /**
     * @brief Returns the process-wide tensor registry.
     *
     * @return A reference to the tensor registry singleton.
     */
    static TensorRegistry& getInstance();

    /**
     * @brief Registers or replaces an entity-view factory.
     *
     * @param[in] engine The physics engine name.
     * @param[in] entityName The entity-view type name.
     * @param[in] factory The factory to register.
     * @return @c true if the key was newly registered; @c false if an existing factory was replaced.
     * @throws std::invalid_argument If @p factory is empty.
     */
    bool registerEntity(const std::string& engine, const std::string& entityName, EntityFactory factory);

    /**
     * @brief Removes an entity-view factory.
     *
     * @param[in] engine The physics engine name.
     * @param[in] entityName The entity-view type name.
     * @return @c true if a factory was removed; @c false if no matching factory was registered.
     */
    bool unregisterEntity(const std::string& engine, const std::string& entityName);

    /**
     * @brief Reports whether an entity-view factory is registered.
     *
     * @param[in] engine The physics engine name.
     * @param[in] entityName The entity-view type name.
     * @return @c true if a matching factory is registered; otherwise, @c false.
     */
    bool hasEntity(const std::string& engine, const std::string& entityName) const;

    /**
     * @brief Creates an entity view with a registered factory.
     *
     * Exceptions raised by the factory propagate to the caller. The factory may call other registry operations.
     *
     * @param[in] engine The physics engine name.
     * @param[in] entityName The entity-view type name.
     * @param[in] paths The USD paths represented by the new entity view.
     * @param[in] options Engine-defined construction arguments, or @c std::nullopt to use the engine's defaults.
     * @return Shared ownership of the entity view returned by the registered factory, or @c nullptr if the factory
     *         returns an empty shared pointer.
     * @throws std::out_of_range If no matching factory is registered.
     */
    std::shared_ptr<IEntityView> createEntity(const std::string& engine,
                                              const std::string& entityName,
                                              const std::vector<std::string>& paths,
                                              const std::optional<EntityOptions>& options = std::nullopt) const;

    /**
     * @brief Declares that a physics engine is available.
     *
     * An engine is a kind of simulator, and the declaration lasts for the life of the process: it describes
     * what this process can simulate, not what it is simulating. A declared engine therefore stays listed
     * once its simulations end, since it can be started again.
     *
     * @param[in] engine The physics engine name.
     * @return `true` for a new declaration; `false` if the engine was already declared.
     */
    bool registerEngine(const std::string& engine);

    /**
     * @brief Lists the declared physics engines.
     *
     * @return The declared engine names in unspecified order.
     */
    std::vector<std::string> listEngines() const;

    /**
     * @brief Lists the simulation names that have registered factories.
     *
     * A simulation name is what @ref createEntity accepts. Each engine registers one under a default name
     * and one per explicitly named simulation, so this reports what can be addressed right now rather than
     * which engines exist.
     *
     * @return The registered simulation names in unspecified order.
     */
    std::vector<std::string> listSimulations() const;

    /**
     * @brief Lists the registered entity-view types for an engine.
     *
     * @param[in] engine The physics engine name.
     * @return The entity-view type names in unspecified order, or an empty vector if @p engine has none.
     */
    std::vector<std::string> listEntities(const std::string& engine) const;

    /**
     * @brief Removes all registered factories for test isolation.
     *
     * @warning Production code and physics engines must not call this function.
     */
    void clearForTesting();

private:
    TensorRegistry() = default;
    ~TensorRegistry() = default;
    TensorRegistry(const TensorRegistry&) = delete;
    TensorRegistry& operator=(const TensorRegistry&) = delete;

    struct Implementation;
    static Implementation& _getImplementationStorage();
    Implementation& _getImplementation();
    const Implementation& _getImplementation() const;
};

} // namespace tensors
} // namespace physics
} // namespace isaacsim
