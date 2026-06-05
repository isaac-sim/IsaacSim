// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ucx/nodes/IUcxNodes.h>

CARB_BINDINGS("isaacsim.ucx.nodes.python")

namespace
{

/**
 * @brief Python bindings for the UCX Nodes module
 *
 * Provides Python interface access to the UCX nodes functionality
 * through pybind11 bindings.
 */
PYBIND11_MODULE(_ucx_nodes, m)
{
    // clang-format off
    using namespace carb;
    using namespace isaacsim::ucx::nodes;

    m.doc() = R"pbdoc(
        Internal interface that is automatically called when the extension is loaded so that Omnigraph nodes are registered.

        Example:

            # import  isaacsim.ucx.nodes.bindings._ucx_nodes as _ucx_nodes

            # Acquire the interface
            interface = _ucx_nodes.acquire_interface()

            # Use the interface
            # ...

            # Release the interface
            _ucx_nodes.release_interface(interface)
    )pbdoc";

    defineInterfaceClass<IUcxNodes>(
        m,
        "IUcxNodes",
        "acquire_interface",
        "release_interface"
    );
}
} // namespace anonymous
