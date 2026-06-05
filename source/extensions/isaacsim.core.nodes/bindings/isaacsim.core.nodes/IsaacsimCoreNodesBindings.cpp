// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/BindingsPythonUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/core/nodes/ICoreNodes.h>

CARB_BINDINGS("isaacsim.core.nodes.python")


namespace
{

PYBIND11_MODULE(_isaacsim_core_nodes, m)
{
    // clang-format off
    using namespace carb;
    using namespace isaacsim::core::nodes;
    m.doc() = R"pbdoc(
        Isaac Sim Core Nodes Module
    )pbdoc";

    defineInterfaceClass< isaacsim::core::nodes::CoreNodes>(m, "CoreNodes", "acquire_interface", "release_interface");


}
}
