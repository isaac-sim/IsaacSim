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

#include <isaacsim/foundation/ovsim/Export.h>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>

#include <string>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace control
{
namespace authoring
{

using ::ovsim::interfaces::control::authoring::InputParameterType;
using ::ovsim::interfaces::control::authoring::OutputParameterType;

/**
 * @brief Create an empty OpenUSD stage and make it the active authoring stage.
 * @return True when the created stage is valid.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool createStage();

/**
 * @brief Open a USD stage and make it the active authoring stage.
 * @param[in] usdPath Path or URL of the USD layer to open.
 * @return True when the opened stage is valid.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool openStage(const std::string& usdPath);

/**
 * @brief Save the active authoring stage.
 * @param[in] usdPath Destination path for the root USD layer.
 * @return True when the stage is saved successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool saveStage(const std::string& usdPath);

/**
 * @brief Import a serialized USD stage and make it the active authoring stage.
 * @param[in] usdString Serialized USDA content.
 * @return True when the imported stage is valid.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool importStageFromString(const std::string& usdString);

/**
 * @brief Export the active authoring stage as USDA text.
 * @return Serialized USDA content.
 */
ISAACSIM_FOUNDATION_OVSIM_API std::string exportStageToString();

/**
 * @brief Release retained stage wrappers, clear cached prim wrappers, and close the active authoring stage.
 *
 * Call `simulation::invalidate()` first when a simulation stage has been initialized so that its retained OVStage
 * instance is closed before the wrapper is released.
 * @return True when the active stage closes successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool closeStage();

/**
 * @brief Add a reference to the active authoring stage.
 * @param[in] usdPath Path or URL of the referenced USD layer.
 * @param[in] path Absolute prim path that receives the reference.
 * @param[in] typeName Prim type to define when @p path does not exist.
 * @return True when the reference is added successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool addReferenceToStage(const std::string& usdPath,
                                                       const std::string& path,
                                                       const std::string& typeName = "Xform");

/**
 * @brief Define a prim on the active authoring stage.
 * @param[in] path Absolute path of the prim to define.
 * @param[in] typeName USD or Isaac Sim prim type to create.
 * @return True when the prim is defined successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool definePrim(const std::string& path, const std::string& typeName = "Xform");

/**
 * @brief Move a prim on the active authoring stage.
 * @param[in] targetPath Absolute path of the prim to move.
 * @param[in] destinationPath Absolute destination path.
 * @return True when the prim is moved successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool movePrim(const std::string& targetPath, const std::string& destinationPath);

/**
 * @brief Remove a prim from the active authoring stage.
 * @param[in] path Absolute path of the prim to remove.
 * @return True when the prim is removed successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool removePrim(const std::string& path);

/**
 * @brief Create an attribute on a prim in the active authoring stage.
 * @param[in] path Absolute path of the target prim.
 * @param[in] attributeName Name of the attribute to create.
 * @param[in] typeName USD type name for the attribute.
 * @return True when the attribute is created successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool createPrimAttribute(const std::string& path,
                                                       const std::string& attributeName,
                                                       const std::string& typeName);

/**
 * @brief Remove an attribute from a prim in the active authoring stage.
 * @param[in] path Absolute path of the target prim.
 * @param[in] attributeName Name of the attribute to remove.
 * @return True when the attribute is removed successfully.
 */
ISAACSIM_FOUNDATION_OVSIM_API bool removePrimAttribute(const std::string& path, const std::string& attributeName);

/**
 * @brief Accept an authoring provider parameter without changing stage state.
 * @param[in] provider Provider identifier reserved for future authoring configuration.
 * @param[in] parameterName Provider-specific parameter name.
 * @param[in] value Parameter value.
 * @note The current implementation accepts and ignores all parameters.
 */
ISAACSIM_FOUNDATION_OVSIM_API void setParameter(const std::string& provider,
                                                const std::string& parameterName,
                                                const InputParameterType& value);

/**
 * @brief Query a retained stage identifier or pointer.
 * @param[in] provider Provider identifier. The supported value is @c stage.
 * @param[in] parameterName Stage property to query. Supported values are @c openusd-stage-id,
 * @c openusd-stage-ptr, @c ovstage-stage-id, and @c ovstage-stage-ptr.
 * @return Requested identifier or pointer. Missing stage identifiers use -1 and missing stage pointers use zero.
 * @throws std::invalid_argument If @p provider or @p parameterName is unsupported.
 */
ISAACSIM_FOUNDATION_OVSIM_API OutputParameterType getParameter(const std::string& provider,
                                                               const std::string& parameterName);

} // namespace authoring
} // namespace control
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim
