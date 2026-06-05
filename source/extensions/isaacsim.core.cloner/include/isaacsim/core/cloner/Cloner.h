// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>
#include <vector>


#ifdef _MSC_VER
#    if ISAACSIM_CORE_CLONER_EXPORT
#        define ISAACSIM_CORE_CLONER_DLL_EXPORT __declspec(dllexport)
#    else
#        define ISAACSIM_CORE_CLONER_DLL_EXPORT __declspec(dllimport)
#    endif
#else
#    define ISAACSIM_CORE_CLONER_DLL_EXPORT
#endif

namespace isaacsim
{
namespace core
{
namespace cloner
{

/**
 * @brief Creates clones of a source prim at specified target paths in the USD stage
 * @details This function performs a fabric clone operation, creating copies of a source prim at multiple
 * target locations within the same stage.
 *
 * @param[in] stageId The unique identifier of the USD stage where cloning will occur
 * @param[in] sourcePrimPath The path to the source prim that will be cloned, this prim should be a valid USD prim
 * @param[in] primPaths Vector of target paths where clones will be created, these prims will be created in Fabric
 * stage not in USD stage
 *
 * @return true if cloning was successful, false otherwise
 *
 * @warning The source prim must exist at the specified path
 */
ISAACSIM_CORE_CLONER_DLL_EXPORT bool fabricClone(long int stageId,
                                                 const std::string& sourcePrimPath,
                                                 const std::vector<std::string>& primPaths);

}
}
}
