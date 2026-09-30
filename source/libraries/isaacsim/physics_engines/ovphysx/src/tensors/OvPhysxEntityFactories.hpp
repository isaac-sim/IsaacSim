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

#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>
#include <ovphysx/ovphysx_types.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

class OvPhysxAdapter;

// Option names read by the entity factories below. Paths alone cannot describe a contact or SDF view, so the
// remaining construction arguments arrive through TensorRegistry's EntityOptions map. An option an engine does
// not recognize is ignored, so passing a contact option to a rigid-body view is not an error.
//
// "filter-patterns"          (rigid-contact)  vector<string>, the contact counterparts to report against.
// "max-contact-data-count"   (rigid-contact)  int64, contact records retained per view.
// "num-points"               (sdf-shape)      int64, query points allocated per selected shape.
extern const char* const g_kFilterPatternsOption;
extern const char* const g_kMaxContactDataCountOption;
extern const char* const g_kPointCountOption;

// Register an adapter so the entity factories can read its ovphysx handle. Called from the bindings
// register() function.
void storeAdapter(int64_t simulationId, std::shared_ptr<OvPhysxAdapter> adapter);
void removeAdapterHandle(int64_t simulationId);

// Construct the view for an entity type against an already-resolved handle. An invalid handle yields an
// unsupported view rather than a broken one.
std::shared_ptr<isaacsim::physics::tensors::EntityView> makeOvPhysxEntityView(
    const std::string& entityType,
    ovphysx_handle_t handle,
    const std::vector<std::string>& paths,
    const std::optional<isaacsim::physics::tensors::EntityOptions>& options = std::nullopt);

// Register the OvPhysX entity factories with TensorRegistry under a simulation name. The name is what callers
// pass to createEntity, and what the factories resolve their handle from, so registering distinct names is how
// several simulations coexist.
void registerOvPhysxEntityFactories(const std::string& simulationName = "ovphysx");

// Unregister the factories registered under a simulation name.
void unregisterOvPhysxEntityFactories(const std::string& simulationName = "ovphysx");

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
