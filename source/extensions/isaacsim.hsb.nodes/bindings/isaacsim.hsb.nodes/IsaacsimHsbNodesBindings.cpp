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

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/hsb/nodes/IHsbNodes.h>

CARB_BINDINGS("isaacsim.hsb.nodes.python")

namespace
{

/**
 * @brief Python bindings for the HSB Nodes plugin
 * @details
 * Provides Python bindings to the IHsbNodes Carbonite interface.
 *
 * When acquire_interface() is called, it triggers carbOnPluginStartup which
 * registers all OmniGraph nodes defined in the plugin.
 */
PYBIND11_MODULE(_hsb_nodes, m)
{
    using namespace carb;
    using namespace isaacsim::hsb::nodes;

    m.doc() = R"pbdoc(
        HSB Nodes Python Bindings

        Provides access to the IHsbNodes Carbonite interface for:
        - Plugin lifecycle management (acquire/release interface)

        This module is primarily used internally by the extension.
    )pbdoc";

    defineInterfaceClass<IHsbNodes>(m, "IHsbNodes", "acquire_interface", "release_interface");
}
} // namespace anonymous
