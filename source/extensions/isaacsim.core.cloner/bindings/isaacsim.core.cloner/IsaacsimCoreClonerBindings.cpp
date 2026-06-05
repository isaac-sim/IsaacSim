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

#include <isaacsim/core/cloner/Cloner.h>

CARB_BINDINGS("isaacsim.core.cloner.python")


namespace
{
PYBIND11_MODULE(_isaac_cloner, m)
{
    using namespace carb;
    using namespace isaacsim::core::cloner;
    // We use carb data types, must import bindings for them
    auto carbModule = py::module::import("carb");

    m.def("_fabric_clone", &fabricClone, R"(
        Creates clones of a source prim at specified target paths in the Fabric stage.
        
        This function performs a fabric clone operation, creating copies of a source prim at multiple
        target locations within the Fabric stage. Unlike regular cloning, these clones are created
        directly in the Fabric stage rather than the USD stage, which can be more efficient for
        certain use cases.

        Args:
            stage_id (int): The unique identifier of the USD stage where cloning will occur
            source_prim_path (str): The path to the source prim that will be cloned. This prim should be a valid USD prim.
            prim_paths (List[str]): List of target paths where clones will be created. These prims will be created in the Fabric stage, not in the USD stage.

        Returns:
            bool: True if cloning was successful, False otherwise.

        Warning:
            The source prim must exist at the specified path.
    )");
}
}
