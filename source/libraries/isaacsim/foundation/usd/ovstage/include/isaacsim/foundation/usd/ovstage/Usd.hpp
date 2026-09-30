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

#pragma once

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/foundation/usd/ovstage/Export.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace ovstage
{

namespace array = isaacsim::common::array;

/**
 * @brief Holds attribute values as strings, nested string arrays, or a typed numeric array.
 */
using AttributeValues = std::variant<std::vector<std::string>, std::vector<std::vector<std::string>>, array::Array>;

// Stage

/**
 * @brief Creates a new, empty in-memory USD stage.
 * @return Stage identifier registered in the stage cache, or -1 on failure.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API int64_t createStage();

/**
 * @brief Opens a USD stage from a URL.
 * @param[in] usdPath Path of the USD file to open.
 * @return Stage identifier registered in the stage cache, or -1 on failure.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API int64_t openStage(const std::string& usdPath);

/**
 * @brief Saves a stage to a USD file.
 * @param[in] stageId Stage identifier returned by createStage() or openStage().
 * @param[in] usdPath Path of the USD file to save.
 * @return True if the stage was saved successfully, false otherwise.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool saveStage(int64_t stageId, const std::string& usdPath);

/**
 * @brief Closes and releases a previously created or opened stage.
 * @param[in] stageId Stage identifier returned by createStage() or openStage().
 * @return True if the stage was found and closed, false otherwise.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool closeStage(int64_t stageId);

/**
 * @brief Serializes the stage's root layer to a USDA string.
 * @param[in] stageId Stage to serialize.
 * @return USDA string representation of the stage, or empty string on failure.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string exportStageToString(int64_t stageId);

/**
 * @brief Creates a new in-memory stage populated from a USDA string.
 * @param[in] usdString USDA string to import.
 * @return Stage identifier registered in the stage cache, or -1 on failure.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API int64_t importStageFromString(const std::string& usdString);

/**
 * @brief Checks whether a stage identifier refers to a live stage.
 * @param[in] stageId Stage identifier to validate.
 * @return True if the stage is open and valid.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool isStageValid(int64_t stageId);

/**
 * @brief Returns an opaque pointer to the underlying ovstage_instance_t struct.
 * @param[in] stageId Stage identifier to get the pointer for.
 * @return Opaque pointer to the ovstage_instance_t struct, or nullptr if the stage is not valid.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void* getStagePtr(int64_t stageId);

/**
 * @brief Defines a prim at the given path, creating ancestor prims as needed.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim to define.
 * @param[in] typeName USD schema type name for the new prim.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void definePrim(int64_t stageId,
                                                    const std::string& path,
                                                    const std::string& typeName = "Xform");

/**
 * @brief Moves a prim from one path to another within the same stage.
 * @param[in] stageId Stage to operate on.
 * @param[in] targetPath Absolute SdfPath of the prim to move.
 * @param[in] destinationPath Absolute SdfPath of the desired new location.
 * @return Tuple of (success, finalDestinationPath). On failure, the path is empty.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::tuple<bool, std::string> movePrim(int64_t stageId,
                                                                           const std::string& targetPath,
                                                                           const std::string& destinationPath);

/**
 * @brief Removes a prim and all of its descendants from the stage.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim to remove.
 * @return True if the prim was removed.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool removePrim(int64_t stageId, const std::string& path);

/**
 * @brief Traverses the prim subtree rooted at @p path, invoking @p callback for each prim.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the root prim to start traversal from.
 * @param[in] callback Invoked with the SdfPath string of each visited prim.
 *                     Return false to stop traversal early.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void traversePrim(int64_t stageId,
                                                      const std::string& path,
                                                      std::function<bool(const std::string&)> callback);

/**
 * @brief Returns the stage's metersPerUnit and kilogramsPerUnit scale factors.
 * @param[in] stageId Stage to query.
 * @return Tuple of (metersPerUnit, kilogramsPerUnit).
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::tuple<float, float> getStageUnits(int64_t stageId);

/**
 * @brief Sets the stage's physical unit scale factors.
 * @param[in] stageId Stage to modify.
 * @param[in] metersPerUnit Meters per scene unit. Unchanged if nullopt.
 * @param[in] kilogramsPerUnit Kilograms per mass unit. Unchanged if nullopt.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setStageUnits(int64_t stageId,
                                                       std::optional<float> metersPerUnit = std::nullopt,
                                                       std::optional<float> kilogramsPerUnit = std::nullopt);

/**
 * @brief Returns the stage's up-axis token (e.g. "Y" or "Z").
 * @param[in] stageId Stage to query.
 * @return Up-axis string.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string getStageUpAxis(int64_t stageId);

/**
 * @brief Sets the stage's up-axis.
 * @param[in] stageId Stage to modify.
 * @param[in] upAxis Up-axis token, e.g. "Y" or "Z".
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setStageUpAxis(int64_t stageId, const std::string& upAxis);

/**
 * @brief Returns the stage's start time code, end time code, and time codes per second.
 * @param[in] stageId Stage to query.
 * @return Tuple of (startTimeCode, endTimeCode, timeCodesPerSecond).
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::tuple<float, float, float> getStageTimeCode(int64_t stageId);

/**
 * @brief Sets the stage's time code range and rate.
 * @param[in] stageId Stage to modify.
 * @param[in] startTimeCode Start of the animation range. Unchanged if nullopt.
 * @param[in] endTimeCode End of the animation range. Unchanged if nullopt.
 * @param[in] timeCodesPerSecond Playback rate in time codes per second. Unchanged if nullopt.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setStageTimeCode(int64_t stageId,
                                                          std::optional<float> startTimeCode = std::nullopt,
                                                          std::optional<float> endTimeCode = std::nullopt,
                                                          std::optional<float> timeCodesPerSecond = std::nullopt);

/**
 * @brief Adds a USD reference arc to a prim, defining it first if it does not exist.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the referencing prim.
 * @param[in] usdPath File path or asset URL of the referenced USD asset.
 * @param[in] typeName USD schema type name used when the prim is created.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void addReferenceToStage(int64_t stageId,
                                                             const std::string& path,
                                                             const std::string& usdPath,
                                                             const std::string& typeName = "Xform");

/**
 * @brief Generates a human-readable representation of the stage hierarchy.
 * @param[in] stageId Stage to represent.
 * @param[in] mode Representation format. Supported values:
 *                 - "tree": indented hierarchy showing parent–child relationships.
 *                 - "list": flat list of all prim paths, one per line.
 * @return String containing the stage representation.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string generateStageRepresentation(int64_t stageId,
                                                                            const std::string& mode = "tree");

/**
 * @brief Finds prim paths on the stage whose path string matches a pattern.
 * @param[in] stageId Stage to search.
 * @param[in] path Pattern string used to match prim paths.
 * @param[in] traverse If true, descends into matched prims to find further matches.
 * @return List of matching absolute SdfPath strings.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::vector<std::string> findMatchingPrimPaths(int64_t stageId,
                                                                                   const std::string& path,
                                                                                   bool traverse = false);

// SdfPath

/**
 * @brief Checks whether a string is a syntactically valid SdfPath.
 * @param[in] path String to validate.
 * @return True if the string can be parsed as an SdfPath.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool isValidPathString(const std::string& path);

// Prim

/**
 * @brief Checks whether the prim at @p path exists and is valid on the stage.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return True if the prim exists and is valid.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool isPrimValid(int64_t stageId, const std::string& path);

/**
 * @brief Returns the name component of a prim's path.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return Name string (last element of the path).
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string getName(int64_t stageId, const std::string& path);

/**
 * @brief Returns the schema type name of a prim (e.g. "Xform", "Mesh").
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return Type name string, or empty if the prim has no type.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string getTypeName(int64_t stageId, const std::string& path);

/**
 * @brief Returns the absolute path of a prim's parent.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return Parent path string, or empty for the pseudo-root.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string getParent(int64_t stageId, const std::string& path);

/**
 * @brief Returns the absolute paths of a prim's direct children.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return List of child path strings.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::vector<std::string> getChildren(int64_t stageId, const std::string& path);

/**
 * @brief Checks whether a prim is or derives from a given schema type.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] schemaType USD schema type name to test against.
 * @return True if the prim's type is or derives from @p schemaType.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool isA(int64_t stageId, const std::string& path, const std::string& schemaType);

/**
 * @brief Checks whether an applied API schema is applied to a prim.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] schemaType Applied API schema type name (SingleApplyAPI or MultipleApplyAPI kind).
 * @param[in] instanceName Instance name for multiple-apply schemas; empty for single-apply.
 * @return True if the API schema is applied.
 * @throws std::invalid_argument if @p schemaType is unknown or is not an applied API schema.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool hasApi(int64_t stageId,
                                                const std::string& path,
                                                const std::string& schemaType,
                                                const std::optional<std::string>& instanceName = std::nullopt);

/**
 * @brief Applies an API schema to a prim.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] schemaType Applied API schema type name (SingleApplyAPI or MultipleApplyAPI kind).
 * @param[in] instanceName Instance name for multiple-apply schemas; empty for single-apply.
 * @return True if the schema was successfully applied.
 * @throws std::invalid_argument if @p schemaType is unknown or is not an applied API schema.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool applyApi(int64_t stageId,
                                                  const std::string& path,
                                                  const std::string& schemaType,
                                                  const std::optional<std::string>& instanceName = std::nullopt);

/**
 * @brief Removes an applied API schema from a prim.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] schemaType Applied API schema type name (SingleApplyAPI or MultipleApplyAPI kind).
 * @param[in] instanceName Instance name for multiple-apply schemas; empty for single-apply.
 * @return True if the schema was removed, false if it was not applied.
 * @throws std::invalid_argument if @p schemaType is unknown or is not an applied API schema.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool removeApi(int64_t stageId,
                                                   const std::string& path,
                                                   const std::string& schemaType,
                                                   const std::optional<std::string>& instanceName = std::nullopt);

/**
 * @brief Returns the list of API schema names applied to a prim.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return List of applied API schema identifier strings.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::vector<std::string> getAppliedSchemas(int64_t stageId, const std::string& path);

/**
 * @brief Returns the variant sets and their available or selected variants for a prim.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] selection If true, returns the selected variant per set; if false, returns all available variants.
 * @return Map from variant set name to a list of variant name(s).
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::unordered_map<std::string, std::vector<std::string>> getVariants(
    int64_t stageId, const std::string& path, bool selection = true);

/**
 * @brief Sets variant selections on a prim's variant sets.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] variants Map from variant set name to the desired variant selection(s).
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setVariants(
    int64_t stageId, const std::string& path, const std::unordered_map<std::string, std::vector<std::string>>& variants);

// Prim Attribute

/**
 * @brief Creates a new attribute on a prim.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] attributeName Name of the attribute to create.
 * @param[in] typeName SdfValueTypeName string for the attribute (e.g. "float", "double3[]").
 * @return True if the attribute was created successfully.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool createPrimAttribute(int64_t stageId,
                                                             const std::string& path,
                                                             const std::string& attributeName,
                                                             const std::string& typeName);

/**
 * @brief Removes an attribute from a prim.
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] attributeName Name of the attribute to remove.
 * @return True if the attribute was removed, false if it did not exist.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API bool removePrimAttribute(int64_t stageId,
                                                             const std::string& path,
                                                             const std::string& attributeName);

/**
 * @brief Returns the SdfValueTypeName string of an attribute (e.g. "float", "double3[]").
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @param[in] attributeName Name of the attribute to inspect.
 * @return Type name string, or empty if the attribute does not exist.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::string getPrimAttributeTypeName(int64_t stageId,
                                                                         const std::string& path,
                                                                         const std::string& attributeName);

/**
 * @brief Returns the names of all attributes on a prim.
 * @param[in] stageId Stage to query.
 * @param[in] path Absolute SdfPath of the prim.
 * @return List of attribute name strings.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::vector<std::string> getPrimAttributeNames(int64_t stageId,
                                                                                   const std::string& path);

/**
 * @brief Reads an attribute's value from multiple prims in a single call.
 * @param[in] stageId Stage to query.
 * @param[in] paths List of absolute SdfPath strings to read from.
 * @param[in] attributeName Name of the attribute to read.
 * @return Attribute values across all requested prims.
 * @throws std::invalid_argument If @p paths is empty, or any path does not refer to a valid prim.
 * @throws isaacsim::common::exceptions::AttributeNameError If any prim lacks @p attributeName.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API AttributeValues getPrimAttributeValues(int64_t stageId,
                                                                           const std::vector<std::string>& paths,
                                                                           const std::string& attributeName);

/**
 * @brief Writes an attribute's value to multiple prims in a single call.
 * @param[in] stageId Stage to operate on.
 * @param[in] paths List of absolute SdfPath strings to write to.
 * @param[in] attributeName Name of the attribute to write.
 * @param[in] values Values to write; must be compatible with the attribute's type.
 * @return Per-prim success flags, one entry per path.
 * @throws std::invalid_argument If @p paths is empty, or any path does not refer to a valid prim.
 * @throws isaacsim::common::exceptions::AttributeNameError If any prim lacks @p attributeName.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::vector<bool> setPrimAttributeValues(int64_t stageId,
                                                                             const std::vector<std::string>& paths,
                                                                             const std::string& attributeName,
                                                                             const AttributeValues& values);

// Xform

/**
 * @brief Ensures a prim has the canonical transform ops order [xformOp:translate, xformOp:orient, xformOp:scale].
 *
 * Removes non-standard transform ops and stray @c :unitsResolve properties.
 * Missing transform ops are created with double precision.
 * The world pose is preserved across the reorganization.
 *
 * After this call callers can set translations, orientations, and scales by writing directly
 * to @c xformOp:translate, @c xformOp:orient, and @c xformOp:scale - without resorting to the
 * destructive @c MakeMatrixXform path.
 *
 * @param[in] stageId Stage to operate on.
 * @param[in] path Absolute SdfPath of the prim to normalize.
 * @throws std::invalid_argument If the prim does not exist or is not Xformable.
 *
 * @warning Currently, ovstage does not support this operation, so calling this function will result in a logic error.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void resetXformOpProperties(int64_t stageId, const std::string& path);

/**
 * @brief Returns the local scales of the given Xformable prims.
 * @param[in] stageId Stage to query.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @return Array of shape ``(N, 3)`` containing ``[sx, sy, sz]`` for each prim.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API array::Array getXformLocalScales(int64_t stageId,
                                                                     const std::vector<std::string>& paths);

/**
 * @brief Sets the local scales of the given Xformable prims.
 * @details Preserves the existing local translation and rotation.
 * @param[in] stageId Stage to operate on.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @param[in] scales Array of shape ``(N, 3)`` containing ``[sx, sy, sz]`` for each prim.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 * @throws std::invalid_argument If any prim does not have the canonical transform ops order [xformOp:translate,
 * xformOp:orient, xformOp:scale].
 * @throws std::invalid_argument If a provided array's row count does not match @p paths size, or any row has the wrong
 * length.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setXformLocalScales(int64_t stageId,
                                                             const std::vector<std::string>& paths,
                                                             const array::Array& scales);

/**
 * @brief Returns the local poses of the given Xformable prims.
 * @param[in] stageId Stage to query.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @return Tuple of two arrays:
 *         - translations of shape ``(N, 3)`` containing ``[x, y, z]`` for each prim;
 *         - orientations of shape ``(N, 4)`` containing xyzw-ordered quaternions for each prim.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::tuple<array::Array, array::Array> getXformLocalPoses(
    int64_t stageId, const std::vector<std::string>& paths);

/**
 * @brief Sets the local poses of the given Xformable prims.
 * @details Only the specified components are updated; unspecified ones are read from the current transform.
 *          Preserves the existing local scale.
 * @param[in] stageId Stage to operate on.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @param[in] translations Optional array of shape ``(N, 3)`` with ``[x, y, z]`` local translations.
 * @param[in] orientations Optional array of shape ``(N, 4)`` with xyzw-ordered local orientations.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 * @throws std::invalid_argument If any prim does not have the canonical transform ops order [xformOp:translate,
 * xformOp:orient, xformOp:scale].
 * @throws std::invalid_argument If a provided array's row count does not match @p paths size, or any row has the wrong
 * length.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setXformLocalPoses(int64_t stageId,
                                                            const std::vector<std::string>& paths,
                                                            const std::optional<array::Array>& translations = std::nullopt,
                                                            const std::optional<array::Array>& orientations = std::nullopt);

/**
 * @brief Returns the world poses of the given Xformable prims.
 * @param[in] stageId Stage to query.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @return Tuple of two arrays:
 *         - positions of shape ``(N, 3)`` containing ``[x, y, z]`` for each prim;
 *         - orientations of shape ``(N, 4)`` containing xyzw-ordered quaternions for each prim.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API std::tuple<array::Array, array::Array> getXformWorldPoses(
    int64_t stageId, const std::vector<std::string>& paths);

/**
 * @brief Sets the world poses of the given Xformable prims.
 * @details Only the specified components are updated; unspecified ones are read from the current world transform.
 *          Preserves the existing local scale.
 * @param[in] stageId Stage to operate on.
 * @param[in] paths List of absolute SdfPath strings of Xformable prims.
 * @param[in] positions Optional array of shape ``(N, 3)`` with ``[x, y, z]`` world positions.
 * @param[in] orientations Optional array of shape ``(N, 4)`` with xyzw-ordered world orientations.
 * @throws std::invalid_argument If any path does not refer to a valid or Xformable prim.
 * @throws std::invalid_argument If any prim does not have the canonical transform ops order [xformOp:translate,
 * xformOp:orient, xformOp:scale].
 * @throws std::invalid_argument If a provided array's row count does not match @p paths size, or any row has the wrong
 * length.
 */
ISAACSIM_FOUNDATION_USD_OVSTAGE_API void setXformWorldPoses(int64_t stageId,
                                                            const std::vector<std::string>& paths,
                                                            const std::optional<array::Array>& positions = std::nullopt,
                                                            const std::optional<array::Array>& orientations = std::nullopt);

} // namespace ovstage
} // namespace usd
} // namespace foundation
} // namespace isaacsim
