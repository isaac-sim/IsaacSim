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

#include <isaacsim/physics/ovsim/Export.h>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>

#include <string>

namespace isaacsim
{
namespace physics
{
namespace ovsim
{
namespace control
{
namespace authoring
{

using ::ovsim::interfaces::control::authoring::InputParameterType;
using ::ovsim::interfaces::control::authoring::OutputParameterType;

/** @brief Report that creating a stage is unavailable in this adapter.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool createStage();

/** @brief Report that opening a stage is unavailable in this adapter.
 * @param[in] usdPath Path or URL of the USD stage to open.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool openStage(const std::string& usdPath);

/** @brief Report that saving a stage is unavailable in this adapter.
 * @param[in] usdPath Destination path or URL for the stage.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool saveStage(const std::string& usdPath);

/** @brief Report that importing stage contents is unavailable in this adapter.
 * @param[in] usdString Serialized USD stage contents.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool importStageFromString(const std::string& usdString);

/** @brief Report that exporting stage contents is unavailable in this adapter.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API std::string exportStageToString();

/** @brief Report that closing a stage is unavailable in this adapter.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool closeStage();

/** @brief Report that adding a USD reference is unavailable in this adapter.
 * @param[in] usdPath Path or URL of the referenced USD asset.
 * @param[in] path Stage path of the prim that receives the reference.
 * @param[in] typeName USD type used when the destination prim must be defined.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool addReferenceToStage(const std::string& usdPath,
                                                    const std::string& path,
                                                    const std::string& typeName = "Xform");

/** @brief Report that defining a prim is unavailable in this adapter.
 * @param[in] path Stage path of the prim to define.
 * @param[in] typeName USD type of the prim.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool definePrim(const std::string& path, const std::string& typeName = "Xform");

/** @brief Report that moving a prim is unavailable in this adapter.
 * @param[in] targetPath Current stage path of the prim.
 * @param[in] destinationPath New stage path for the prim.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool movePrim(const std::string& targetPath, const std::string& destinationPath);

/** @brief Report that removing a prim is unavailable in this adapter.
 * @param[in] path Stage path of the prim to remove.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool removePrim(const std::string& path);

/** @brief Report that creating a prim attribute is unavailable in this adapter.
 * @param[in] path Stage path of the prim.
 * @param[in] attributeName Name of the attribute to create.
 * @param[in] typeName USD type name of the attribute.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool createPrimAttribute(const std::string& path,
                                                    const std::string& attributeName,
                                                    const std::string& typeName);

/** @brief Report that removing a prim attribute is unavailable in this adapter.
 * @param[in] path Stage path of the prim.
 * @param[in] attributeName Name of the attribute to remove.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API bool removePrimAttribute(const std::string& path, const std::string& attributeName);

/** @brief Report that setting an authoring parameter is unavailable in this adapter.
 * @param[in] provider Name of the parameter provider.
 * @param[in] parameterName Name of the parameter to set.
 * @param[in] value Value to assign to the parameter.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API void setParameter(const std::string& provider,
                                             const std::string& parameterName,
                                             const InputParameterType& value);

/** @brief Report that reading an authoring parameter is unavailable in this adapter.
 * @param[in] provider Name of the parameter provider.
 * @param[in] parameterName Name of the parameter to retrieve.
 * @return This function does not return.
 * @throws std::logic_error Always; stage authoring is not implemented.
 */
ISAACSIM_PHYSICS_OVSIM_API OutputParameterType getParameter(const std::string& provider,
                                                            const std::string& parameterName);

} // namespace authoring
} // namespace control
} // namespace ovsim
} // namespace physics
} // namespace isaacsim
