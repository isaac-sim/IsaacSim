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

#include <isaacsim/hsb/core/IHsbCore.h>

CARB_BINDINGS("isaacsim.hsb.core.python")

namespace
{

/**
 * @brief Python bindings for the HSB Core plugin
 * @details
 * Provides Python bindings to the IHsbCore Carbonite interface.
 *
 * This module is used by:
 * - Extension lifecycle (extension.py) to load/unload the native plugin
 *
 * When acquire_interface() is called, it triggers carbOnPluginStartup.
 */
PYBIND11_MODULE(_hsb_core, m)
{
    using namespace carb;
    using namespace isaacsim::hsb::core;

    m.doc() = R"pbdoc(
        HSB Core Python Bindings

        Provides access to the IHsbCore Carbonite interface for:
        - Plugin lifecycle management (acquire/release interface)

        This module is primarily used internally by the extension.
    )pbdoc";

    defineInterfaceClass<IHsbCore>(m, "IHsbCore", "acquire_interface", "release_interface");
}
} // namespace anonymous
