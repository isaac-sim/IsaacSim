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

#include "OvPhysxEntityFactories.hpp"

#include "../OvPhysxAdapter.hpp"
#include "OvPhysxEntityViews.hpp"

#include <isaacsim/physics/registration/Physics.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>
#include <ovphysx/ovphysx_types.h>

#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

const char* const g_kFilterPatternsOption = "filter-patterns";
const char* const g_kMaxContactDataCountOption = "max-contact-data-count";
const char* const g_kPointCountOption = "num-points";

// Process-global map: simulationId -> OvPhysxAdapter.
// Stores the adapter so the entity factories can read the ovphysx C handle.
static std::mutex g_adapterMapMutex;
static std::unordered_map<int64_t, std::shared_ptr<OvPhysxAdapter>> g_adapterMap;


void storeAdapter(int64_t simulationId, std::shared_ptr<OvPhysxAdapter> adapter)
{
    std::lock_guard<std::mutex> lock(g_adapterMapMutex);
    g_adapterMap[simulationId] = std::move(adapter);
}

void removeAdapterHandle(int64_t simulationId)
{
    std::lock_guard<std::mutex> lock(g_adapterMapMutex);
    g_adapterMap.erase(simulationId);
}

static std::shared_ptr<OvPhysxAdapter> lookupAdapter(int64_t simulationId)
{
    std::lock_guard<std::mutex> lock(g_adapterMapMutex);
    auto adapterIterator = g_adapterMap.find(simulationId);
    return (adapterIterator != g_adapterMap.end()) ? adapterIterator->second : nullptr;
}

// Entity types this engine offers through TensorRegistry::createEntity.
constexpr const char* g_kEntityTypes[] = {
    "articulation",        "rigid-body", "rigid-contact", "volume-deformable-body", "surface-deformable-body",
    "deformable-material", "sdf-shape",
};

// Capacities used when the caller states none. A contact or SDF view built without the matching option starts
// here and is resized by the first read that asks for a different extent.
constexpr int g_kDefaultContactDataCount = 1000;
constexpr int g_kDefaultQueryPointCount = 1;

namespace
{
// Read an integral option. An option of the wrong type is a caller mistake rather than a silent fallback, so it
// is rejected instead of being replaced by the default.
int readIntegerOption(const std::optional<isaacsim::physics::tensors::EntityOptions>& options,
                      const char* optionName,
                      int defaultValue)
{
    if (!options)
    {
        return defaultValue;
    }
    const auto optionIterator = options->find(optionName);
    if (optionIterator == options->end())
    {
        return defaultValue;
    }
    const int64_t* value = std::get_if<int64_t>(&optionIterator->second);
    if (!value)
    {
        throw std::invalid_argument(std::string("createEntity: option '") + optionName + "' must be an integer");
    }
    // These options size engine allocations, so a value the engine's `int` cannot hold is rejected rather
    // than wrapped into a small or negative capacity.
    if (*value < 0 || *value > std::numeric_limits<int>::max())
    {
        throw std::invalid_argument(std::string("createEntity: option '") + optionName + "' must be in [0, " +
                                    std::to_string(std::numeric_limits<int>::max()) + "] (received " +
                                    std::to_string(*value) + ")");
    }
    return static_cast<int>(*value);
}

// Read a string-list option. A single string is accepted as a one-element list, which is how a caller naming one
// filter would reasonably write it.
std::vector<std::string> readStringListOption(const std::optional<isaacsim::physics::tensors::EntityOptions>& options,
                                              const char* optionName)
{
    if (!options)
    {
        return {};
    }
    const auto optionIterator = options->find(optionName);
    if (optionIterator == options->end())
    {
        return {};
    }
    if (const auto* values = std::get_if<std::vector<std::string>>(&optionIterator->second))
    {
        return *values;
    }
    if (const auto* value = std::get_if<std::string>(&optionIterator->second))
    {
        return { *value };
    }
    throw std::invalid_argument(std::string("createEntity: option '") + optionName +
                                "' must be a string or a list of strings");
}
} // namespace

// Entity factories receive paths and options only, so the handle comes from the simulation registered under the
// name the factory was registered with. Naming each simulation distinctly is what lets several coexist: the
// lookup is exact, so nothing is inferred. Returns OVPHYSX_INVALID_HANDLE when no simulation of that name is
// active, which yields an unsupported view rather than a broken one.
static ovphysx_handle_t getActiveSimulationHandle(const std::string& simulationName)
{
    namespace registration = isaacsim::physics::registration;
    const registration::SimulationId activeSimulationId = registration::getActiveSimulationId(simulationName);
    if (activeSimulationId == registration::g_kInvalidSimulationId)
    {
        return OVPHYSX_INVALID_HANDLE;
    }
    const std::shared_ptr<OvPhysxAdapter> adapter = lookupAdapter(static_cast<int64_t>(activeSimulationId.id));
    return adapter ? adapter->getHandle() : OVPHYSX_INVALID_HANDLE;
}

static std::shared_ptr<isaacsim::physics::tensors::EntityView> makeOvPhysxEntityViewForHandle(
    const std::string& entityType,
    ovphysx_handle_t handle,
    const std::vector<std::string>& paths,
    const std::optional<isaacsim::physics::tensors::EntityOptions>& options);

std::shared_ptr<isaacsim::physics::tensors::EntityView> makeOvPhysxEntityView(
    const std::string& entityType,
    ovphysx_handle_t handle,
    const std::vector<std::string>& paths,
    const std::optional<isaacsim::physics::tensors::EntityOptions>& options)
{
    std::shared_ptr<isaacsim::physics::tensors::EntityView> view =
        makeOvPhysxEntityViewForHandle(entityType, handle, paths, options);
    // A view whose patterns matched nothing binds no operations at all, and a caller cannot tell that from a
    // registration someone forgot to write. Say "no support" instead, as the contact and SDF views already do
    // when their own construction cannot proceed.
    if (view && view->listImplementations(isaacsim::physics::tensors::ImplementationKind::eGet).empty() &&
        view->listImplementations(isaacsim::physics::tensors::ImplementationKind::eSet).empty())
    {
        return makeUnsupportedView(paths.empty() ? std::string() : paths.front(), entityType);
    }
    return view;
}

static std::shared_ptr<isaacsim::physics::tensors::EntityView> makeOvPhysxEntityViewForHandle(
    const std::string& entityType,
    ovphysx_handle_t handle,
    const std::vector<std::string>& paths,
    const std::optional<isaacsim::physics::tensors::EntityOptions>& options)
{
    if (handle == OVPHYSX_INVALID_HANDLE)
    {
        return makeUnsupportedView(paths.empty() ? std::string() : paths.front(), entityType);
    }
    // One path is a pattern the engine expands itself; several are resolved here and bound as explicit prim
    // paths, concatenated in caller order with first-wins de-duplication. Keeping the single-path case in
    // pattern mode is what lets a view report the glob it was built from rather than the paths it matched.
    const bool singlePattern = paths.size() == 1;
    if (entityType == "articulation")
    {
        return singlePattern ? std::make_shared<OvPhysxArticulationEntityView>(handle, paths.front()) :
                               std::make_shared<OvPhysxArticulationEntityView>(handle, paths);
    }
    if (entityType == "rigid-body")
    {
        return singlePattern ? std::make_shared<OvPhysxRigidBodyEntityView>(handle, paths.front()) :
                               std::make_shared<OvPhysxRigidBodyEntityView>(handle, paths);
    }
    if (entityType == "volume-deformable-body")
    {
        return singlePattern ? std::make_shared<OvPhysxVolumeDeformableBodyEntityView>(handle, paths.front()) :
                               std::make_shared<OvPhysxVolumeDeformableBodyEntityView>(handle, paths);
    }
    if (entityType == "surface-deformable-body")
    {
        return singlePattern ? std::make_shared<OvPhysxSurfaceDeformableBodyEntityView>(handle, paths.front()) :
                               std::make_shared<OvPhysxSurfaceDeformableBodyEntityView>(handle, paths);
    }
    if (entityType == "deformable-material")
    {
        return singlePattern ? std::make_shared<OvPhysxDeformableMaterialEntityView>(handle, paths.front()) :
                               std::make_shared<OvPhysxDeformableMaterialEntityView>(handle, paths);
    }
    // Contact and SDF views take a single pattern; there is no path-list form to fall back on, so a
    // multi-path request is rejected rather than silently narrowed to the first entry.
    if (paths.size() > 1)
    {
        throw std::invalid_argument("createEntity: entity '" + entityType + "' accepts a single path (received " +
                                    std::to_string(paths.size()) + ")");
    }
    const std::string pattern = paths.empty() ? std::string() : paths.front();
    if (entityType == "rigid-contact")
    {
        return std::make_shared<OvPhysxRigidContactEntityView>(
            handle, pattern, readStringListOption(options, g_kFilterPatternsOption),
            readIntegerOption(options, g_kMaxContactDataCountOption, g_kDefaultContactDataCount));
    }
    if (entityType == "sdf-shape")
    {
        return std::make_shared<OvPhysxSdfShapeEntityView>(
            handle, pattern, readIntegerOption(options, g_kPointCountOption, g_kDefaultQueryPointCount));
    }
    throw std::out_of_range("createEntity: unknown ovphysx entity type '" + entityType + "'");
}

// ----------------------------------------------------------------------------
// TensorRegistry factory registration
// ----------------------------------------------------------------------------

// Registered under "ovphysx" (no longer under "_ovphysx_cpp") since entity
// views expose all supported methods directly from C++. The Python tensors
// layer only applies _attach_warp_dispatch to wrap returns as wp.array.
void registerOvPhysxEntityFactories(const std::string& simulationName)
{
    using namespace isaacsim::physics::tensors;
    // The engine is a kind of simulator and is declared once; the factories below are keyed by simulation
    // name, of which there may be several.
    TensorRegistry::getInstance().registerEngine("ovphysx");

    for (const char* entityType : g_kEntityTypes)
    {
        TensorRegistry::getInstance().registerEntity(
            simulationName, entityType,
            [entityType, simulationName](const std::vector<std::string>& paths,
                                         const std::optional<EntityOptions>& options) -> std::shared_ptr<EntityView>
            { return makeOvPhysxEntityView(entityType, getActiveSimulationHandle(simulationName), paths, options); });
    }
}

void unregisterOvPhysxEntityFactories(const std::string& simulationName)
{
    using namespace isaacsim::physics::tensors;
    for (const char* entityType : g_kEntityTypes)
    {
        TensorRegistry::getInstance().unregisterEntity(simulationName, entityType);
    }
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
