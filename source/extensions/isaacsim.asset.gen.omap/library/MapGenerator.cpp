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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "isaacsim/core/includes/ScopedTimer.h"

#include <extensions/PxSceneQueryExt.h>
#include <isaacsim/asset/gen/omap/MapGenerator.h>
#include <octomap/octomap.h>
#include <omni/physx/IPhysx.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <PxActor.h>
#include <PxArticulationJointReducedCoordinate.h>
#include <PxArticulationLink.h>
#include <PxArticulationReducedCoordinate.h>
#include <PxPhysicsAPI.h>
#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <stack>

namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace omap
{

namespace
{

/**
 * @brief Finds the PhysX scene associated with the given USD stage
 * @details
 * Searches through all prims in the stage to find a UsdPhysicsScene prim
 * and retrieves the associated PhysX scene pointer.
 *
 * @param[in] physXPtr Pointer to the PhysX interface
 * @param[in] stagePtr Pointer to the USD stage
 *
 * @return Pointer to the PhysX scene, or nullptr if not found
 *
 * @pre physXPtr must be a valid pointer to the PhysX interface
 * @pre stagePtr must be a valid USD stage
 */
::physx::PxScene* findPhysxScene(omni::physx::IPhysx* physXPtr, pxr::UsdStageWeakPtr stagePtr)
{
    pxr::UsdPrimRange range = stagePtr->Traverse();
    ::physx::PxScene* physxScene = nullptr;
    for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        pxr::UsdPrim prim = *iter;

        if (prim.IsA<pxr::UsdPhysicsScene>())
        {

            physxScene = static_cast<::physx::PxScene*>(
                physXPtr->getPhysXPtr(prim.GetPrimPath(), omni::physx::PhysXType::ePTScene));

            if (physxScene)
            {
                return physxScene;
            }
        }
    }
    return nullptr;
}

} // anonymous namespace

/**
 * @brief Constructs a new MapGenerator instance
 * @details
 * Initializes the generator with PhysX interface and USD stage references.
 * Creates a new octree with the default cell size and configures its parameters.
 * Then finds the associated PhysX scene from the USD stage.
 *
 * @param[in] physXPtr Pointer to the PhysX interface for collision detection
 * @param[in] stagePtr Pointer to the USD stage containing the scene geometry
 *
 * @pre physXPtr must be a valid pointer to the PhysX interface
 * @pre stagePtr must be a valid USD stage
 *
 * @post m_tree will be initialized with a new octree
 * @post m_physxScenePtr will be set to the PhysX scene if found
 *
 * @note The octree is configured with default occupancy threshold (0.5),
 *       probability hit (0.7), and clamping threshold (0.1)
 */
MapGenerator::MapGenerator(omni::physx::IPhysx* physXPtr, pxr::UsdStageWeakPtr stagePtr)
{
    m_physx = physXPtr;
    m_stage = stagePtr;
    m_tree = new octomap::OcTree(m_cellSize);
    m_physxScenePtr = findPhysxScene(m_physx, m_stage);
    if (!m_physxScenePtr)
    {
        CARB_LOG_ERROR("Physics scene not found in stage");
        return;
    }

    m_tree->setOccupancyThres(0.5);
    m_tree->setProbHit(0.7);
    m_tree->setClampingThresMin(0.1);
}

/**
 * @brief Destructor for MapGenerator
 * @details
 * Cleans up resources used by the generator, specifically
 * deallocating the octree to prevent memory leaks.
 *
 * @post m_tree will be deleted
 */
MapGenerator::~MapGenerator()
{
    delete m_tree;
    m_tree = nullptr;
}

/**
 * @brief Updates the octree map settings
 * @details
 * Configures the parameters of the occupancy map, including cell size and
 * the numerical values used to represent different occupancy states.
 * The cell size determines the resolution of the map and is applied
 * to the underlying octree structure.
 *
 * @param[in] cellSize Size of each cell in the octree (meters)
 * @param[in] occupiedValue Value representing occupied cells (default: 1.0)
 * @param[in] unoccupiedValue Value representing unoccupied cells (default: 0.0)
 * @param[in] unknownValue Value representing unknown cells (default: 0.5)
 *
 * @pre cellSize should be positive
 * @post The octree resolution will be updated to match the new cell size
 *
 * @note Smaller cell sizes provide higher resolution but require more memory
 */
void MapGenerator::updateSettings(const float cellSize,
                                  const float occupiedValue,
                                  const float unoccupiedValue,
                                  const float unknownValue)
{
    m_cellSize = cellSize;
    m_occupiedValue = occupiedValue;
    m_unoccupiedValue = unoccupiedValue;
    m_unknownValue = unknownValue;
    m_tree->setResolution(m_cellSize);
}

/**
 * @brief Sets the transform parameters for the map generation
 * @details
 * Defines the origin and boundaries of the area to be mapped in world coordinates.
 * The boundaries are adjusted to align with the cell grid by rounding the min/max
 * points to the nearest cell boundaries.
 *
 * @param[in] inputOrigin Origin point of the map in world coordinates
 * @param[in] inputMinPoint Minimum point of the map relative to origin
 * @param[in] inputMaxPoint Maximum point of the map relative to origin
 *
 * @post m_inputOrigin, m_inputMinPoint, and m_inputMaxPoint will be updated
 *
 * @note The min/max points are adjusted to align with cell boundaries
 *       by rounding down/up to the nearest cell size multiple
 */
void MapGenerator::setTransform(carb::Float3 inputOrigin, carb::Float3 inputMinPoint, carb::Float3 inputMaxPoint)
{
    carb::Float3 roundedMin = {
        std::floor(inputMinPoint.x / m_cellSize) * m_cellSize,
        std::floor(inputMinPoint.y / m_cellSize) * m_cellSize,
        std::floor(inputMinPoint.z / m_cellSize) * m_cellSize,
    };

    carb::Float3 roundedMax = {
        std::ceil(inputMaxPoint.x / m_cellSize) * m_cellSize,
        std::ceil(inputMaxPoint.y / m_cellSize) * m_cellSize,
        std::ceil(inputMaxPoint.z / m_cellSize) * m_cellSize,
    };

    m_inputOrigin = inputOrigin;
    m_inputMinPoint = roundedMin;
    m_inputMaxPoint = roundedMax;
}

/**
 * @brief Generates a 2D occupancy map using PhysX overlap queries
 * @details
 * Creates a 2D projection of the occupancy map by performing overlap tests
 * at each cell in the XY plane. The method uses a tall box that extends
 * in the Z direction to detect obstacles at any height. The octree is updated
 * with both occupied and free cells based on the collision test results.
 *
 * @pre m_physxScenePtr must be valid
 * @pre m_tree must be valid
 *
 * @post m_tree will be populated with occupancy information
 * @post The octree's inner nodes will be updated for consistency
 *
 * @note This method is more efficient than generate3d() but provides less spatial information
 * @warning If the PhysX scene or octree is not initialized, the function will return early
 */
void MapGenerator::generate2d()
{
    if (!m_physxScenePtr)
    {
        CARB_LOG_ERROR("Physics scene not initialized");
        return;
    }

    if (!m_tree)
    {
        CARB_LOG_ERROR("Octree not initialized");
        return;
    }

    // Clear existing octree data
    m_tree->clear();

    // Create overlap test geometry
    // Use a tall box that extends in Z direction to detect obstacles at any height
    float geomHeight = ::physx::PxAbs(m_inputMaxPoint.z - m_inputMinPoint.z) / 2.0f + m_cellSize / 2.0f;
    ::physx::PxBoxGeometry cellGeom(::physx::PxVec3(m_cellSize / 2.0f, m_cellSize / 2.0f, geomHeight));

    // Sets to store occupied and unoccupied cell keys
    octomap::KeySet occupiedCells;
    octomap::KeySet unoccupiedCells;
    ::physx::PxOverlapHit hit;

    // Iterate through XY grid
    for (float ix = m_inputMinPoint.x + m_cellSize / 2.0f; ix <= m_inputMaxPoint.x - m_cellSize / 2.0f; ix += m_cellSize)
    {
        for (float iy = m_inputMinPoint.y + m_cellSize / 2.0f; iy <= m_inputMaxPoint.y - m_cellSize / 2.0f;
             iy += m_cellSize)
        {
            // Convert world coordinates to octree key
            octomap::OcTreeKey key =
                m_tree->coordToKey(octomap::point3d(ix + m_inputOrigin.x, iy + m_inputOrigin.y, m_inputOrigin.z));

            // Position the overlap test box
            float height = m_inputOrigin.z + m_inputMinPoint.z + (m_inputMaxPoint.z - m_inputMinPoint.z) / 2.0f;
            ::physx::PxTransform pose(::physx::PxVec3(ix + m_inputOrigin.x, iy + m_inputOrigin.y, height));

            // Perform overlap test and store result
            if (::physx::PxSceneQueryExt::overlapAny(*m_physxScenePtr, cellGeom, pose, hit))
            {
                occupiedCells.insert(key);
            }
            else
            {
                unoccupiedCells.insert(key);
            }
        }
    }

    // Update octree with unoccupied cells
    for (const auto& key : unoccupiedCells)
    {
        m_tree->updateNode(key, false, true);
    }

    // Update octree with occupied cells
    for (const auto& key : occupiedCells)
    {
        m_tree->updateNode(key, true, true);
    }

    // Update internal nodes of the octree
    m_tree->updateInnerOccupancy();
}

/**
 * @brief Generates a 3D occupancy map using PhysX overlap queries
 * @details
 * Creates a full 3D volumetric representation of the occupancy map by
 * performing overlap tests at each cell in the XYZ volume. The method uses
 * a cube-shaped test geometry to detect collisions with scene objects.
 * The octree is updated with both occupied and free cells based on the
 * collision test results.
 *
 * @pre m_physxScenePtr must be valid
 * @pre m_tree must be valid
 *
 * @post m_tree will be populated with 3D occupancy information
 * @post The octree's inner nodes will be updated for consistency
 *
 * @note This method provides more complete spatial information than generate2d()
 * @warning This method requires more memory and computation than generate2d()
 * @warning If the PhysX scene or octree is not initialized, the function will return early
 */
void MapGenerator::generate3d()
{
    if (!m_physxScenePtr)
    {
        CARB_LOG_ERROR("Physics scene not initialized");
        return;
    }

    if (!m_tree)
    {
        CARB_LOG_ERROR("Octree not initialized");
        return;
    }

    // Clear existing octree data
    m_tree->clear();

    // Create overlap test geometry
    // Use a cube with half extents equal to half the cell size
    ::physx::PxBoxGeometry cellGeom(::physx::PxVec3(m_cellSize / 2.0f, m_cellSize / 2.0f, m_cellSize / 2.0f));

    // Sets to store occupied and unoccupied cell keys
    octomap::KeySet occupiedCells, unoccupiedCells;
    ::physx::PxOverlapHit hit;

    // Iterate through XYZ grid
    for (float ix = m_inputMinPoint.x + m_cellSize / 2.0f; ix <= m_inputMaxPoint.x - m_cellSize / 2.0f; ix += m_cellSize)
    {
        for (float iy = m_inputMinPoint.y + m_cellSize / 2.0f; iy <= m_inputMaxPoint.y - m_cellSize / 2.0f;
             iy += m_cellSize)
        {
            for (float iz = m_inputMinPoint.z + m_cellSize / 2.0f; iz <= m_inputMaxPoint.z - m_cellSize / 2.0f;
                 iz += m_cellSize)
            {
                // Convert world coordinates to octree key
                octomap::OcTreeKey key = m_tree->coordToKey(
                    octomap::point3d(ix + m_inputOrigin.x, iy + m_inputOrigin.y, iz + m_inputOrigin.z));

                // Position the overlap test cube
                ::physx::PxTransform pose(
                    ::physx::PxVec3(ix + m_inputOrigin.x, iy + m_inputOrigin.y, iz + m_inputOrigin.z));

                // Perform overlap test and store result
                if (::physx::PxSceneQueryExt::overlapAny(*m_physxScenePtr, cellGeom, pose, hit))
                {
                    occupiedCells.insert(key);
                }
                else
                {
                    unoccupiedCells.insert(key);
                }
            }
        }
    }

    // Update octree with unoccupied cells
    for (octomap::KeySet::iterator iter = unoccupiedCells.begin(); iter != unoccupiedCells.end(); ++iter)
    {
        m_tree->updateNode(*iter, false, true);
    }

    // Update octree with occupied cells
    for (octomap::KeySet::iterator iter = occupiedCells.begin(); iter != occupiedCells.end(); ++iter)
    {
        m_tree->updateNode(*iter, true, true);
    }

    // Update internal nodes of the octree
    m_tree->updateInnerOccupancy();
}

/**
 * @brief Returns the positions of all occupied cells in the octree
 * @details
 * Iterates through all leaf nodes in the octree and collects the world coordinates
 * of cells that are marked as occupied. The coordinates represent the center points
 * of each occupied cell.
 *
 * @return Vector of 3D positions (Float3) for occupied cells
 *
 * @pre m_tree should be valid
 *
 * @note Returns an empty vector if the octree is not initialized
 * @warning For large maps with small cell sizes, this vector can be very large
 */
std::vector<carb::Float3> MapGenerator::getOccupiedPositions()
{
    std::vector<carb::Float3> pos;
    if (m_tree)
    {
        auto beginLeafIter = m_tree->begin_leafs();
        auto endLeafIter = m_tree->end_leafs();
        for (octomap::OcTree::leaf_iterator it = beginLeafIter, end = endLeafIter; it != end; ++it)
        {
            if (m_tree->isNodeOccupied(&(*it)))
            {
                pos.push_back(carb::Float3({ it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z() }));
            }
        }
    }
    return pos;
}

/**
 * @brief Returns the positions of all unoccupied (free) cells in the octree
 * @details
 * Iterates through all leaf nodes in the octree and collects the world coordinates
 * of cells that are marked as unoccupied (free). The coordinates represent the
 * center points of each free cell.
 *
 * @return Vector of 3D positions (Float3) for free cells
 *
 * @pre m_tree should be valid
 *
 * @note The returned vector may be very large for maps with many free cells
 * @warning For large maps with small cell sizes, this function may be memory intensive
 */
std::vector<carb::Float3> MapGenerator::getFreePositions()
{
    std::vector<carb::Float3> pos;
    auto beginLeafIter = m_tree->begin_leafs();
    auto endLeafIter = m_tree->end_leafs();
    for (octomap::OcTree::leaf_iterator it = beginLeafIter, end = endLeafIter; it != end; ++it)
    {
        if (!m_tree->isNodeOccupied(&(*it)))
        {
            pos.push_back(carb::Float3({ it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z() }));
        }
    }
    return pos;
}

/**
 * @brief Gets the minimum bound of the octree in world coordinates
 * @details
 * Retrieves the minimum x, y, and z coordinates that define the
 * bounding box of the octree map in world space.
 *
 * @return Minimum bound as Float3 (x,y,z)
 *
 * @pre m_tree should be valid
 *
 * @note Returns (0,0,0) if the octree is not initialized
 */
carb::Float3 MapGenerator::getMinBound()
{
    double x = 0, y = 0, z = 0;
    if (m_tree)
    {
        m_tree->getMetricMin(x, y, z);
    }
    return carb::Float3({ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) });
}

/**
 * @brief Gets the maximum bound of the octree in world coordinates
 * @details
 * Retrieves the maximum x, y, and z coordinates that define the
 * bounding box of the octree map in world space.
 *
 * @return Maximum bound as Float3 (x,y,z)
 *
 * @pre m_tree should be valid
 *
 * @note Returns (0,0,0) if the octree is not initialized
 */
carb::Float3 MapGenerator::getMaxBound()
{

    double x = 0, y = 0, z = 0;
    if (m_tree)
    {
        m_tree->getMetricMax(x, y, z);
    }
    return carb::Float3({ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) });
}

/**
 * @brief Calculates the dimensions of the octree in number of cells
 * @details
 * Determines how many cells the occupancy map contains along each axis
 * by dividing the metric size of the map by the cell size.
 *
 * @return Number of cells in each dimension as Int3 (x,y,z)
 *
 * @pre m_tree should be valid
 *
 * @note Returns (0,0,0) if the octree is not initialized
 */
carb::Int3 MapGenerator::getDimensions()
{
    carb::Int3 numCells = { 0, 0, 0 };

    if (m_tree)
    {
        // min and max in meters
        carb::Float3 min = getMinBound();
        carb::Float3 max = getMaxBound();
        carb::Float3 size = { max.x - min.x, max.y - min.y, max.z - min.z };
        // scale by the grid resolution to get the number of pixels
        // num_cells = meters / (meters/cell)
        numCells = { static_cast<int>(size.x / m_cellSize), static_cast<int>(size.y / m_cellSize),
                     static_cast<int>(size.z / m_cellSize) };
    }
    return numCells;
}

// Direction vectors for 4-way connectivity (up, right, down, left)
int g_row[] = { -1, 0, 1, 0 };
int g_col[] = { 0, 1, 0, -1 };

/**
 * @brief Checks if a given cell position is valid and matches the target value
 * @details
 * Validates whether a specified position is within the grid boundaries
 * and whether the cell at that position has the target value.
 *
 * @param[in] buffer The grid buffer to check
 * @param[in] numCells Dimensions of the grid (width, height)
 * @param[in] x X coordinate to check
 * @param[in] y Y coordinate to check
 * @param[in] target Value we're looking to match
 *
 * @return true if position is valid and matches target value, false otherwise
 *
 * @pre buffer must be a valid pointer to a grid buffer
 */
bool isSafe(const float* buffer, carb::Int2 numCells, int x, int y, float target)
{
    if (x < 0 || x >= numCells.x || y < 0 || y >= numCells.y)
    {
        return false;
    }
    size_t index = y * numCells.x + x;
    return buffer[index] == target;
}

/**
 * @brief Performs a flood fill operation using Depth-First Search
 * @details
 * Starting from position (sx,sy), replaces all connected cells matching the target value
 * with the replacement value. Uses 4-way connectivity (up, right, down, left).
 * The algorithm uses depth-first search with a stack to avoid recursion stack overflow.
 *
 * @param[in,out] buffer The grid buffer to modify
 * @param[in] numCells Dimensions of the grid (width, height)
 * @param[in] sx Starting X coordinate
 * @param[in] sy Starting Y coordinate
 * @param[in] replacement Value to fill connected regions with
 *
 * @pre buffer must be a valid pointer to a grid buffer
 * @pre (sx,sy) must be a valid position within the grid
 *
 * @post All cells connected to (sx,sy) with the same initial value will be changed to replacement
 */
void floodfill(float* buffer, carb::Int2 numCells, int sx, int sy, float replacement)
{
    // Get the target value we're replacing at the start position
    size_t startIndex = sy * numCells.x + sx;
    float target = buffer[startIndex];

    // Stack for DFS - stores coordinates as (x,y) pairs
    std::stack<std::pair<int, int>> stack;
    stack.push({ sx, sy });

    // Process cells until stack is empty
    while (!stack.empty())
    {
        // Get next cell to process
        auto [x, y] = stack.top();
        stack.pop();

        // Fill current cell
        size_t index = y * numCells.x + x;
        buffer[index] = replacement;

        // Check all 4 neighboring cells
        for (int k = 0; k < 4; k++)
        {
            int newX = x + g_col[k];
            int newY = y + g_row[k];

            // If neighbor is valid and matches target, add to stack
            if (isSafe(buffer, numCells, newX, newY, target))
            {
                stack.push({ newX, newY });
            }
        }
    }
}

/**
 * @brief Generates a 2D grid representation of the octree map
 * @details
 * Creates a 2D projection of the occupancy map as a grid of values.
 * The grid is initialized with unknown values, then occupied cells are marked,
 * and finally a flood fill is performed from the origin to mark free spaces.
 * Each cell in the grid contains one of three values:
 * - m_occupiedValue: Cell contains an obstacle
 * - m_unoccupiedValue: Cell is known to be free space
 * - m_unknownValue: Cell state is unknown
 *
 * @return Vector containing the grid representation as float values
 *
 * @pre m_tree should be valid
 *
 * @note Returns an empty vector if the octree is not initialized or dimensions are invalid
 * @note The buffer is stored in row-major order with y as rows and x as columns
 */
std::vector<float> MapGenerator::getBuffer()
{
    std::vector<float> buffer;
    if (!m_tree)
    {
        return buffer;
    }

    // Get map bounds and dimensions
    carb::Float3 min = getMinBound();
    carb::Float3 max = getMaxBound();
    carb::Int3 numCells = getDimensions();

    // Validate grid size
    if (numCells.x * numCells.y <= 0)
    {
        return buffer;
    }

    // Initialize buffer with unknown values
    buffer.resize(numCells.x * numCells.y);
    std::fill(buffer.begin(), buffer.end(), m_unknownValue);

    // Mark occupied cells in the buffer
    for (auto it = m_tree->begin_leafs(); it != m_tree->end_leafs(); ++it)
    {
        if (m_tree->isNodeOccupied(&(*it)))
        {
            // Convert 3D world coordinates to 2D grid coordinates
            size_t index =
                static_cast<size_t>(it.getCoordinate().y() / m_cellSize - min.y / m_cellSize) * numCells.x +
                static_cast<size_t>((-it.getCoordinate().x() + min.x + max.x) / m_cellSize - min.x / m_cellSize);

            buffer[index] = m_occupiedValue;
        }
    }

    // Calculate starting point for flood fill (robot's position)
    carb::Int2 startPos = { static_cast<int>(-m_inputOrigin.x / m_cellSize + max.x / m_cellSize),
                            static_cast<int>(m_inputOrigin.y / m_cellSize - min.y / m_cellSize) };

    // Fill known free space from robot's position
    floodfill(buffer.data(), { numCells.x, numCells.y }, startPos.x, startPos.y, m_unoccupiedValue);

    return buffer;
}

/**
 * @brief Converts the 2D grid representation into an RGBA color buffer
 * @details
 * Creates a visualization of the occupancy map by assigning colors to different cell states.
 * Each cell is colored according to its occupancy state:
 * - Occupied cells: Use the occupied color (typically red)
 * - Unoccupied cells: Use the unoccupied color (typically green or blue)
 * - Unknown cells: Use the unknown color (typically gray)
 *
 * @param[in] occupied RGBA color values for occupied cells
 * @param[in] unoccupied RGBA color values for unoccupied cells
 * @param[in] unknown RGBA color values for unknown cells
 *
 * @return Vector containing RGBA color values (4 bytes per cell)
 *
 * @pre m_tree should be valid
 *
 * @note Returns an empty vector if the octree is not initialized
 * @note The buffer is stored in row-major order with 4 bytes per cell (RGBA)
 */
std::vector<char> MapGenerator::getColoredByteBuffer(const carb::Int4& occupied,
                                                     const carb::Int4& unoccupied,
                                                     const carb::Int4& unknown)
{
    std::vector<char> colorBuffer;
    if (!m_tree)
    {
        return colorBuffer;
    }

    // Get the grid representation
    std::vector<float> buffer = getBuffer();

    // Allocate color buffer (4 bytes per cell for RGBA)
    colorBuffer.resize(buffer.size() * 4);

    // Convert grid values to RGBA colors
    for (size_t i = 0; i < buffer.size(); i++)
    {
        // Calculate base index for RGBA values
        size_t rgbaIndex = i * 4;

        // Select color based on cell state
        const carb::Int4* color;
        if (buffer[i] == m_unknownValue)
        {
            color = &unknown;
        }
        else if (buffer[i] == m_unoccupiedValue)
        {
            color = &unoccupied;
        }
        else // m_occupiedValue
        {
            color = &occupied;
        }

        // Copy RGBA values to buffer
        colorBuffer[rgbaIndex + 0] = color->x;
        colorBuffer[rgbaIndex + 1] = color->y;
        colorBuffer[rgbaIndex + 2] = color->z;
        colorBuffer[rgbaIndex + 3] = color->w;
    }

    return colorBuffer;
}
}
}
}
}
