// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "NanobindHelpers.hpp"

#include <isaacsim/physics/manager/PhysicsInteraction.hpp>
#include <isaacsim/physics/registration/simulator/Interaction.hpp>
#include <isaacsim/physics/registration/simulator/InteractionTools.hpp>
#include <nanobind/stl/unordered_map.h>

namespace nb = nanobind;

void isaacsim::physics::manager::details::bindPhysicsInteraction(nb::module_& module)
{
    using namespace isaacsim::physics::manager;
    using namespace isaacsim::physics::registration;

    // Expose DebugDataItemType values as a SimpleNamespace so Python callers
    // can still do `_physics.DebugDataItemType.FLOAT`.
    nb::object typeHolder = nb::module_::import_("types").attr("SimpleNamespace")(
        nb::arg("FLOAT") = static_cast<int>(DebugDataItemType::eFloat),
        nb::arg("VECTOR") = static_cast<int>(DebugDataItemType::eVector),
        nb::arg("POINT") = static_cast<int>(DebugDataItemType::ePoint),
        nb::arg("QUATERNION") = static_cast<int>(DebugDataItemType::eQuaternion),
        nb::arg("STRING") = static_cast<int>(DebugDataItemType::eString),
        nb::arg("BOOL") = static_cast<int>(DebugDataItemType::eBoolean),
        nb::arg("INT") = static_cast<int>(DebugDataItemType::eInteger),
        nb::arg("UNDEFINED") = static_cast<int>(DebugDataItemType::eUndefined));
    module.attr("DebugDataItemType") = typeHolder;

    module.def(
        "handle_raycast",
        [](nb::object originObject, nb::object directionObject, bool inputActive)
        {
            const Float3 origin = originObject.is_none() ? Float3{} : nb::cast<Float3>(originObject);
            const Float3 direction = directionObject.is_none() ? Float3{} : nb::cast<Float3>(directionObject);
            handleRaycast(origin, direction, inputActive);
        },
        nb::arg("origin").none(), nb::arg("direction").none(), nb::arg("input"),
        "Forward an interactive raycast request to active simulations.");
    module.def(
        "get_prim_debug_data",
        [](const std::string& primPath) -> nb::dict {
            return isaacsim::physics::registration::details::convertDebugDataToPythonDictionary(
                getPrimDebugData(primPath));
        },
        nb::arg("prim_path"), "Aggregate debug data across all active simulations for the given prim.");
}
