// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Defines.h>
#include <carb/Types.h>
#if defined(_WIN32)
#    include <PxPhysicsAPI.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wpragmas"
#    include <PxPhysicsAPI.h>
#    pragma GCC diagnostic pop
#endif

namespace octomap
{
class OcTree;
}

namespace physx
{
class PxShape;
class PxRigidStatic;
class PxMaterial;
class PxScene;
}

namespace omni
{
namespace physx
{
class IPhysx;
}
}
namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace omap
{
#ifdef _MSC_VER
#    if ISAACSIM_ASSET_GEN_OMAP_EXPORT
#        define DLL_EXPORT __declspec(dllexport)
#    else
#        define DLL_EXPORT __declspec(dllimport)
#    endif
#else
#    define DLL_EXPORT
#endif

/**
 * @class MapGenerator
 * @brief Generator class for creating 2D and 3D occupancy maps from USD stages
 * @details
 * The MapGenerator class provides functionality to generate occupancy maps from USD stage data.
 * It supports both 2D and 3D map generation with configurable cell sizes and occupancy values.
 * The generator uses PhysX for collision detection and octomap for spatial representation.
 *
 * Occupancy maps are useful for robotic applications like motion planning, navigation,
 * and spatial reasoning about environments. The maps represent space as either occupied,
 * free, or unknown, which allows algorithms to make decisions about valid paths and actions.
 *
 * @note This class requires valid PhysX and USD stage pointers to function properly
 * @warning Memory usage increases with smaller cell sizes and larger map volumes
 */
class DLL_EXPORT MapGenerator
{
public:
    /**
     * @brief Constructs a new MapGenerator instance
     * @details
     * Initializes the generator with PhysX interface and USD stage references.
     * The PhysX interface is used for collision detection, and the USD stage
     * provides the geometry of the environment to be mapped.
     *
     * @param[in] physx Pointer to the PhysX interface for collision detection
     * @param[in] stage Pointer to the USD stage containing the scene geometry
     *
     * @pre physx pointer must be valid and initialized
     * @pre stage must be a valid USD stage
     *
     * @throws std::runtime_error If the PhysX scene cannot be created or accessed
     */
    MapGenerator(omni::physx::IPhysx* physx, pxr::UsdStageWeakPtr stage);

    /**
     * @brief Destructor for MapGenerator
     * @details
     * Cleans up resources used by the generator, including the octomap tree.
     */
    ~MapGenerator();

    /**
     * @brief Updates the generator's occupancy map settings
     * @details
     * Configures the cell size and values used for different occupancy states.
     * These values determine the map resolution and how different cell states
     * are represented numerically in the occupancy buffer.
     *
     * @param[in] cellSize Size of each occupancy grid cell in meters
     * @param[in] occupiedValue Numerical value assigned to occupied cells (default: 1.0)
     * @param[in] unoccupiedValue Numerical value assigned to unoccupied cells (default: 0.0)
     * @param[in] unknownValue Numerical value assigned to cells with unknown occupancy (default: 0.5)
     *
     * @pre cellSize must be positive
     *
     * @note Smaller cell sizes provide higher resolution but require more memory and computation time
     */
    void updateSettings(float cellSize, float occupiedValue, float unoccupiedValue, float unknownValue);

    /**
     * @brief Sets the transformation parameters for map generation
     * @details
     * Defines the origin and bounds of the area to be mapped.
     * The occupancy map will be generated within the volume defined by these parameters.
     *
     * @param[in] inputOrigin Origin point in world coordinates
     * @param[in] minPoint Minimum bounds relative to origin, defining the lower corner of the map volume
     * @param[in] maxPoint Maximum bounds relative to origin, defining the upper corner of the map volume
     *
     * @pre minPoint components must be less than maxPoint components
     *
     * @note The volume defined should encompass the relevant parts of the scene to be mapped
     */
    void setTransform(carb::Float3 inputOrigin, carb::Float3 minPoint, carb::Float3 maxPoint);

    /**
     * @brief Generates a 2D occupancy map
     * @details
     * Creates a 2D projection of the scene's occupancy by collapsing the 3D space
     * onto a horizontal plane. This is useful for applications like 2D navigation or
     * floor plan generation.
     *
     * @post The internal occupancy data structures will be populated with 2D occupancy information
     *
     * @note This method is more efficient than generate3d() but provides less spatial information
     */
    void generate2d();

    /**
     * @brief Generates a 3D occupancy map
     * @details
     * Creates a full 3D volumetric representation of the scene's occupancy.
     * This provides complete spatial information about the environment,
     * useful for 3D path planning and spatial reasoning.
     *
     * @post The internal occupancy data structures will be populated with 3D occupancy information
     *
     * @note This method requires more memory and computation than generate2d()
     */
    void generate3d();

    /**
     * @brief Retrieves positions of all occupied cells
     * @details
     * Gets the world coordinates of all cells that are marked as occupied
     * in the current occupancy map.
     *
     * @return Vector of 3D positions representing the centers of occupied cells
     *
     * @note The returned positions are in world coordinates
     */
    std::vector<carb::Float3> getOccupiedPositions();

    /**
     * @brief Retrieves positions of all free (unoccupied) cells
     * @details
     * Gets the world coordinates of all cells that are marked as free (unoccupied)
     * in the current occupancy map.
     *
     * @return Vector of 3D positions representing the centers of free cells
     *
     * @note The returned positions are in world coordinates
     * @warning For large maps with small cell sizes, this vector can be very large
     */
    std::vector<carb::Float3> getFreePositions();

    /**
     * @brief Gets the minimum boundary point of the map
     * @details
     * Retrieves the minimum coordinate values of the occupancy map's bounding box
     * in world coordinates.
     *
     * @return Minimum boundary coordinates as Float3
     */
    carb::Float3 getMinBound();

    /**
     * @brief Gets the maximum boundary point of the map
     * @details
     * Retrieves the maximum coordinate values of the occupancy map's bounding box
     * in world coordinates.
     *
     * @return Maximum boundary coordinates as Float3
     */
    carb::Float3 getMaxBound();

    /**
     * @brief Gets the dimensions of the map in cell units
     * @details
     * Retrieves the number of cells along each axis of the occupancy map.
     * This represents the resolution of the map in each dimension.
     *
     * @return Number of cells in each dimension as Int3
     */
    carb::Int3 getDimensions();

    /**
     * @brief Retrieves the raw occupancy buffer
     * @details
     * Gets the raw occupancy values for all cells in the map.
     * The values correspond to the occupancy state of each cell,
     * as configured in updateSettings().
     *
     * @return Vector of occupancy values for all cells
     *
     * @note The buffer is stored in row-major order (x, then y, then z)
     */
    std::vector<float> getBuffer();

    /**
     * @brief Generates a colored visualization buffer
     * @details
     * Creates an RGBA buffer for visualization purposes, where each cell
     * is colored according to its occupancy state (occupied, unoccupied, or unknown).
     *
     * @param[in] occupied Color for occupied cells (RGBA)
     * @param[in] unoccupied Color for unoccupied cells (RGBA)
     * @param[in] unknown Color for unknown cells (RGBA)
     *
     * @return Vector of bytes representing RGBA values for each cell
     *
     * @note The buffer is stored in row-major order (x, then y, then z) with 4 bytes per cell (RGBA)
     */
    std::vector<char> getColoredByteBuffer(const carb::Int4& occupied,
                                           const carb::Int4& unoccupied,
                                           const carb::Int4& unknown);

private:
    /**
     * @brief Cell size in meters
     * @details Controls the resolution of the occupancy map
     */
    float m_cellSize = 0.05f;

    /**
     * @brief Pointer to PhysX interface
     * @details Used for collision detection during map generation
     */
    omni::physx::IPhysx* m_physx = nullptr;

    /**
     * @brief Weak pointer to USD stage
     * @details References the scene geometry used for map generation
     */
    pxr::UsdStageWeakPtr m_stage = nullptr;

    /**
     * @brief Parent USD primitive
     * @details Used for organizing created USD elements
     */
    pxr::UsdPrim m_parentPrim;

    /**
     * @brief Pointer to octomap tree structure
     * @details Stores the occupancy map data in an efficient hierarchical structure
     */
    octomap::OcTree* m_tree = nullptr;

    /**
     * @brief Pointer to PhysX scene
     * @details Used for collision queries during map generation
     */
    ::physx::PxScene* m_physxScenePtr = nullptr;

    /**
     * @brief Origin point for map generation
     * @details The reference point in world coordinates for the map
     */
    carb::Float3 m_inputOrigin;

    /**
     * @brief Minimum point relative to origin
     * @details Defines the lower corner of the map volume
     */
    carb::Float3 m_inputMinPoint;

    /**
     * @brief Maximum point relative to origin
     * @details Defines the upper corner of the map volume
     */
    carb::Float3 m_inputMaxPoint;

    /**
     * @brief Value assigned to occupied cells
     * @details Used when generating the occupancy buffer
     */
    float m_occupiedValue = 1.0f;

    /**
     * @brief Value assigned to unoccupied cells
     * @details Used when generating the occupancy buffer
     */
    float m_unoccupiedValue = 0.0f;

    /**
     * @brief Value assigned to unknown cells
     * @details Used when generating the occupancy buffer
     */
    float m_unknownValue = 0.5f;
};

}
}
}
}
