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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <isaacsim/core/includes/Defines.h>
#include <pxr/usd/usdPhysics/scene.h>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

/**
 * @class PhysicsScene
 * @brief High level wrapper for manipulating a USD Physics Scene prim and its attributes.
 * @details
 * This class provides a convenient interface for working with USD Physics Scene prims.
 * It wraps the underlying pxr::UsdPhysicsScene and provides methods for accessing
 * physics-related properties such as gravity.
 *
 * The class can be constructed from a string path, SdfPath, or UsdPrim. If the prim
 * at the specified path does not exist, a new PhysicsScene prim will be created.
 * If the prim exists but is not a PhysicsScene, an exception will be thrown.
 *
 * @note The class uses the default USD context's stage for all operations.
 */
class ISAACSIM_EXPORT PhysicsScene
{
public:
    /**
     * @brief Constructs a PhysicsScene from a string path.
     * @details
     * If a prim exists at the specified path, it must be a PhysicsScene prim.
     * If no prim exists at the path, a new PhysicsScene prim will be created.
     *
     * @param[in] path The USD path to the physics scene prim.
     *
     * @throws std::invalid_argument If a prim exists at the path but is not a PhysicsScene.
     */
    PhysicsScene(const std::string& path);

    /**
     * @brief Constructs a PhysicsScene from an SdfPath.
     * @details
     * Converts the SdfPath to a string and delegates to the string constructor.
     *
     * @param[in] path The SdfPath to the physics scene prim.
     *
     * @throws std::invalid_argument If a prim exists at the path but is not a PhysicsScene.
     */
    PhysicsScene(const pxr::SdfPath& path)
    {
        PhysicsScene(path.GetString());
    }

    /**
     * @brief Constructs a PhysicsScene from a UsdPrim.
     * @details
     * Extracts the path from the UsdPrim and delegates to the string constructor.
     *
     * @param[in] prim The UsdPrim representing the physics scene.
     *
     * @throws std::invalid_argument If the prim is not a PhysicsScene.
     */
    PhysicsScene(const pxr::UsdPrim& prim)
    {
        PhysicsScene(prim.GetPath().GetString());
    }

    /**
     * @brief Gets the USD path to the physics scene prim.
     * @return Const reference to the path string.
     */
    const std::string& getPath()
    {
        return m_path;
    }

    /**
     * @brief Gets the underlying UsdPrim.
     * @return Const reference to the UsdPrim.
     */
    const pxr::UsdPrim& getPrim()
    {
        return m_prim;
    }

    /**
     * @brief Gets the underlying UsdPhysicsScene schema.
     * @return Const reference to the UsdPhysicsScene.
     */
    const pxr::UsdPhysicsScene& getPhysicsScene()
    {
        return m_physicsScene;
    }

    /**
     * @brief Gets the gravity vector for the physics scene.
     * @details
     * Computes the gravity vector by multiplying the gravity direction by the
     * gravity magnitude and adjusting for the stage's meters-per-unit scale.
     *
     * @return The gravity vector in world units per second squared.
     */
    pxr::GfVec3d getGravity();

    /**
     * @brief Checks if the physics scene prim is still valid.
     * @details
     * Returns true if the underlying USD prim is valid and active.
     * This can return false if the prim was deleted, the layer containing
     * it was removed, or the stage was closed.
     *
     * @return True if the prim is valid and active, false otherwise.
     */
    bool isValid() const;

private:
    /** @brief Path to the USD Physics Scene prim. */
    std::string m_path;

    /** @brief The underlying UsdPrim for the physics scene. */
    pxr::UsdPrim m_prim;

    /** @brief The UsdPhysicsScene schema applied to the prim. */
    pxr::UsdPhysicsScene m_physicsScene;
};

/**
 * @brief Gets the paths of all PhysicsScene prims in the default stage.
 * @details
 * Traverses all prims in the default USD context's stage and returns
 * the paths of any prims that are of type UsdPhysicsScene.
 *
 * @return A vector of string paths to all PhysicsScene prims in the stage.
 */
std::vector<std::string> ISAACSIM_EXPORT getPhysicsScenePaths();

/**
 * @brief Gets the paths of all PhysicsScene prims in the specified stage.
 * @details
 * Traverses all prims in the stage identified by the given stage ID and returns
 * the paths of any prims that are of type UsdPhysicsScene.
 *
 * @param[in] stageId The ID of the USD stage to search (from UsdStageCache).
 *
 * @return A vector of string paths to all PhysicsScene prims in the stage.
 *         Returns an empty vector if the stage ID is invalid.
 */
std::vector<std::string> ISAACSIM_EXPORT getPhysicsScenePaths(uint64_t stageId);

/**
 * @brief Gets the paths of all PhysicsScene prims in the specified stage.
 * @details
 * Traverses all prims in the given stage and returns the paths of any prims
 * that are of type UsdPhysicsScene.
 *
 * @param[in] stage Pointer to the USD stage to search.
 *
 * @return A vector of string paths to all PhysicsScene prims in the stage.
 */
std::vector<std::string> ISAACSIM_EXPORT getPhysicsScenePaths(const pxr::UsdStagePtr& stage);


} // namespace simulation_manager
} // namespace core
} // namespace isaacsim
