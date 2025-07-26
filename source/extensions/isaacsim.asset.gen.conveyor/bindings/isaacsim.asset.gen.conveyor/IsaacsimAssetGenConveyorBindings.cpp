// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/asset/gen/conveyor/IOmniIsaacConveyor.h>

CARB_BINDINGS("isaacsim.asset.gen.conveyor.python")

namespace
{

PYBIND11_MODULE(_isaacsim_asset_gen_conveyor, m)
{
    using namespace carb;

    m.doc() = R"pbdoc(
        Internal interface that is automatically called when the extension is loaded so that Omnigraph nodes are registered.
    )pbdoc";

    defineInterfaceClass<isaacsim::asset::gen::conveyor::IOmniIsaacConveyor>(
        m, "IOmniIsaacConveyor", "acquire_interface", "release_interface");
}

} // anonymous namespace
