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

#include "BindingsPhysics.hpp"

#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nb = nanobind;
using namespace isaacsim::physics::tensors;

void isaacsim::physics::manager::details::bindTensorCreate(nb::module_& module)
{
    module.def(
        "create_entity",
        [](const std::string& engine, const std::string& entityName, const std::vector<std::string>& paths,
           const std::optional<EntityOptions>& options)
        {
            return std::static_pointer_cast<EntityView>(
                TensorRegistry::getInstance().createEntity(engine, entityName, paths, options));
        },
        nb::arg("engine"), nb::arg("entity_name"), nb::arg("paths"), nb::arg("options") = nb::none(),
        "Create an entity view using a registered engine factory.");
}
