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

#include <isaacsim/foundation/objects/Export.h>

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

/**
 * @class Stage
 * @brief Wrapper around a USD stage providing lifecycle and scene-level operations.
 * @details
 * Manages a single USD stage identified by a numeric stage ID. Provides methods to open,
 * create, save, and close stages, as well as operations on prims within the stage (define,
 * move, remove, add references) and stage-level metadata (up axis, units, time codes).
 *
 * When constructed without a stage ID an invalid stage is created.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Stage
{
public:
    /**
     * @brief Construct a Stage wrapper.
     * @param[in] backend The backend of the stage, either "openusd" or "ovstage".
     * @param[in] stageId Numeric identifier of an existing USD stage. If not provided,
     *                    an invalid stage is created.
     */
    Stage(std::string backend, std::optional<int64_t> stageId = std::nullopt);
    ~Stage();

    /**
     * @brief Return the backend of the stage.
     * @return The backend of the stage, either "openusd" or "ovstage".
     */
    std::string getBackend() const;

    /**
     * @brief Return the numeric ID of the underlying USD stage.
     * @return The stage identifier.
     */
    int64_t getStageId() const;

    /**
     * @brief Return a pointer to the underlying stage instance.
     * @return A pointer to the underlying stage instance, or @c nullptr if the stage is not valid.
     */
    void* getStagePtr() const;

    /**
     * @brief Check whether the stage is open and accessible.
     * @return @c true if the stage is valid, @c false otherwise.
     */
    bool isValid() const;

    /**
     * @brief Open an existing USD file.
     * @param[in] usdPath URI of the USD file to open.
     * @param[in] makeDefault Whether to make this stage the default stage.
     * @return Reference to this Stage to allow method chaining.
     */
    Stage& openStage(const std::string& usdPath, bool makeDefault = true);

    /**
     * @brief Create a new, empty USD stage, optionally seeded from a template.
     * @param[in] templateName Name of the stage template to use as a starting point.
     *                         If not provided, a blank stage is created.
     * @param[in] makeDefault Whether to make this stage the default stage.
     * @return Reference to this Stage to allow method chaining.
     */
    Stage& createStage(const std::optional<std::string>& templateName = std::nullopt, bool makeDefault = true);

    /**
     * @brief Save the current stage to a USD file.
     * @param[in] usdPath Destination URI.
     * @return @c true if the save succeeded, @c false otherwise.
     */
    bool saveStage(const std::string& usdPath);

    /**
     * @brief Serialize the stage's root layer to a USDA string.
     * @return USDA string representation of the stage, or empty string on failure.
     */
    std::string exportStageToString() const;

    /**
     * @brief Populate this stage from a USDA string.
     * @param[in] usdString USDA string to import.
     * @param[in] makeDefault Whether to make this stage the default stage.
     * @return Reference to this Stage to allow method chaining.
     */
    Stage& importStageFromString(const std::string& usdString, bool makeDefault = true);

    /**
     * @brief Close the current stage and release its resources.
     * @return @c true if the stage was successfully closed, @c false otherwise.
     */
    bool closeStage();

    /**
     * @brief Add a USD reference to an existing file at the specified prim path.
     * @param[in] usdPath URI of the USD file to reference.
     * @param[in] path Stage path at which the reference prim is created.
     * @param[in] primType USD type to assign to the reference prim (default: @c "Xform").
     * @param[in] variants Optional map of variant set names to variant selections to apply
     *                     when the reference is loaded.
     * @return @c true if the reference was added successfully, @c false otherwise.
     */
    bool addReference(const std::string& usdPath,
                      const std::string& path,
                      const std::string& primType = "Xform",
                      const std::optional<std::unordered_map<std::string, std::string>>& variants = std::nullopt) const;

    /**
     * @brief Define a new prim on the stage at the given path.
     * @param[in] path Absolute stage path for the new prim.
     * @param[in] typeName USD schema type name (default: @c "Xform").
     * @return The stage path of the newly defined prim.
     */
    std::string definePrim(const std::string& path, const std::string& typeName = "Xform") const;

    /**
     * @brief Move a prim from one stage path to another.
     * @param[in] targetPath Current absolute path of the prim to move.
     * @param[in] destinationPath Absolute destination path.
     * @return A pair where the first element indicates success and the second is the
     *         final path of the prim after the move.
     */
    std::tuple<bool, std::string> movePrim(const std::string& targetPath, const std::string& destinationPath) const;

    /**
     * @brief Remove a prim and all its descendants from the stage.
     * @param[in] path Absolute stage path of the prim to remove.
     * @return @c true if the prim was removed successfully, @c false otherwise.
     */
    bool removePrim(const std::string& path) const;

    /**
     * @brief Get the stage's up-axis token (e.g. @c "Y" or @c "Z").
     * @return The up-axis token string.
     */
    std::string getUpAxis() const;

    /**
     * @brief Set the stage's up-axis.
     * @param[in] upAxis Up-axis token, typically @c "Y" or @c "Z".
     */
    void setUpAxis(const std::string& upAxis) const;

    /**
     * @brief Get the stage's unit scale factors.
     * @return A pair of (@c metersPerUnit, @c kilogramsPerUnit).
     */
    std::tuple<float, float> getUnits() const;

    /**
     * @brief Set the stage's unit scale factors.
     * @param[in] metersPerUnit  Meters per scene unit. If not provided, the current value is unchanged.
     * @param[in] kilogramsPerUnit Kilograms per scene mass unit. If not provided, the current value is unchanged.
     */
    void setUnits(std::optional<float> metersPerUnit = std::nullopt,
                  std::optional<float> kilogramsPerUnit = std::nullopt) const;

    /**
     * @brief Get the stage's time code metadata.
     * @return A tuple of (@c startTimeCode, @c endTimeCode, @c timeCodesPerSecond).
     */
    std::tuple<float, float, float> getTimeCode() const;

    /**
     * @brief Set the stage's time code metadata.
     * @param[in] startTimeCode      Start time code. If not provided, the current value is unchanged.
     * @param[in] endTimeCode        End time code. If not provided, the current value is unchanged.
     * @param[in] timeCodesPerSecond Time codes per second. If not provided, the current value is unchanged.
     */
    void setTimeCode(std::optional<float> startTimeCode = std::nullopt,
                     std::optional<float> endTimeCode = std::nullopt,
                     std::optional<float> timeCodesPerSecond = std::nullopt) const;

    /**
     * @brief Generate a string representation of the stage hierarchy.
     * @param[in] mode Formatting mode (e.g. @c "tree" for an indented tree view, @c "list" for a flat list).
     * @return A human-readable string describing the stage contents.
     */
    std::string generateStringRepresentation(const std::string& mode = "tree") const;

private:
    struct StageOperations
    {
        int64_t (*createStage)();
        int64_t (*openStage)(const std::string&);
        bool (*saveStage)(int64_t, const std::string&);
        bool (*closeStage)(int64_t);
        std::string (*exportStageToString)(int64_t);
        int64_t (*importStageFromString)(const std::string&);
        bool (*isStageValid)(int64_t);
        void* (*getStagePtr)(int64_t);
        void (*definePrim)(int64_t, const std::string&, const std::string&);
        std::tuple<bool, std::string> (*movePrim)(int64_t, const std::string&, const std::string&);
        bool (*removePrim)(int64_t, const std::string&);
        std::tuple<float, float> (*getStageUnits)(int64_t);
        void (*setStageUnits)(int64_t, std::optional<float>, std::optional<float>);
        std::string (*getStageUpAxis)(int64_t);
        void (*setStageUpAxis)(int64_t, const std::string&);
        std::tuple<float, float, float> (*getStageTimeCode)(int64_t);
        void (*setStageTimeCode)(int64_t, std::optional<float>, std::optional<float>, std::optional<float>);
        void (*addReferenceToStage)(int64_t, const std::string&, const std::string&, const std::string&);
        std::string (*generateStageRepresentation)(int64_t, const std::string&);
    };

    void _validate() const;
    static StageOperations _populateStageOps(const std::string& backend);

    int64_t m_stageId;
    std::string m_backend;
    StageOperations m_stageOps;
};

namespace details
{

ISAACSIM_FOUNDATION_OBJECTS_API void setDefaultStage(std::optional<Stage> stage);
ISAACSIM_FOUNDATION_OBJECTS_API Stage getDefaultStage();
ISAACSIM_FOUNDATION_OBJECTS_API void setActiveStage(std::optional<Stage> stage);
ISAACSIM_FOUNDATION_OBJECTS_API Stage getActiveStage();

class ISAACSIM_FOUNDATION_OBJECTS_API StageGuard
{
public:
    explicit StageGuard(Stage stage);
    ~StageGuard();

    StageGuard(const StageGuard&) = delete;
    StageGuard& operator=(const StageGuard&) = delete;
    StageGuard(StageGuard&&) = delete;
    StageGuard& operator=(StageGuard&&) = delete;

private:
    std::optional<Stage> m_previous;
};

} // namespace details

} // namespace objects
} // namespace foundation
} // namespace isaacsim
