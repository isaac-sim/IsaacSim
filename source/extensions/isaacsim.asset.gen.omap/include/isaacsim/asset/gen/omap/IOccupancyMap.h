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

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace omap
{

/**
 * @brief Interface for occupancy map generation
 * @details
 * This interface provides functionality for generating and managing occupancy maps.
 * Occupancy maps represent spatial information about the environment as a grid of cells,
 * where each cell indicates whether the corresponding space is occupied, free, or unknown.
 *
 * @note The interface is designed to work with both 2D and 3D occupancy maps
 */
struct OccupancyMap
{
    CARB_PLUGIN_INTERFACE("isaacsim::asset::gen::omap::OccupancyMap", 0, 1);

    /**
     * @brief Generates the occupancy map
     * @details
     * Initializes and creates an occupancy map based on the current scene.
     * This should be called after setting the transform and cell size parameters.
     *
     * @pre Transform and cell size must be set before calling this function
     * @post A new occupancy map will be generated and stored in memory
     */
    void(CARB_ABI* generateMap)();

    /**
     * @brief Updates the visualization
     * @details
     * Refreshes the visual representation of the occupancy map.
     * This includes drawing the bounding box, grid, and coordinate axes.
     *
     * @note This should be called whenever the map or visualization settings change
     */
    void(CARB_ABI* update)();

    /**
     * @brief Sets the transform for map generation
     * @details
     * Defines the origin and boundaries for the occupancy map in world coordinates.
     *
     * @param[in] inputOrigin Origin point in world coordinates, serves as the reference point
     * @param[in] minPoint Minimum bounds relative to origin, defines the lower corner of the map volume
     * @param[in] maxPoint Maximum bounds relative to origin, defines the upper corner of the map volume
     *
     * @pre minPoint components should be less than maxPoint components
     */
    void(CARB_ABI* setTransform)(carb::Float3 inputOrigin, carb::Float3 minPoint, carb::Float3 maxPoint);

    /**
     * @brief Sets the cell size for the map
     * @details
     * Configures the resolution of the occupancy map by setting the size of each cell.
     *
     * @param[in] cellSize Size of each cell in meters
     *
     * @pre cellSize must be positive
     * @note Smaller cell sizes will result in higher resolution maps but require more memory
     */
    void(CARB_ABI* setCellSize)(float cellSize);

    /**
     * @brief Gets positions of occupied cells
     * @details
     * Retrieves the world positions of all cells that are marked as occupied.
     *
     * @return Vector of 3D positions representing the centers of occupied cells
     */
    std::vector<carb::Float3>(CARB_ABI* getOccupiedPositions)();

    /**
     * @brief Gets positions of free cells
     * @details
     * Retrieves the world positions of all cells that are marked as free (unoccupied).
     *
     * @return Vector of 3D positions representing the centers of free cells
     */
    std::vector<carb::Float3>(CARB_ABI* getFreePositions)();

    /**
     * @brief Gets minimum bounds of the map
     * @details
     * Retrieves the minimum coordinate values of the occupancy map's bounding box.
     *
     * @return Minimum bounds as Float3 in world coordinates
     */
    carb::Float3(CARB_ABI* getMinBound)();

    /**
     * @brief Gets maximum bounds of the map
     * @details
     * Retrieves the maximum coordinate values of the occupancy map's bounding box.
     *
     * @return Maximum bounds as Float3 in world coordinates
     */
    carb::Float3(CARB_ABI* getMaxBound)();

    /**
     * @brief Gets dimensions of the map in cells
     * @details
     * Retrieves the number of cells along each axis of the occupancy map.
     *
     * @return Dimensions as Int3, where each component represents the number of cells along that axis
     */
    carb::Int3(CARB_ABI* getDimensions)();

    /**
     * @brief Gets the occupancy buffer
     * @details
     * Retrieves the raw occupancy values for all cells in the map.
     *
     * @return Vector of cell values, where each value indicates the occupancy state
     *         (typically 1.0 for occupied, 0.0 for free, and 0.5 for unknown)
     */
    std::vector<float>(CARB_ABI* getBuffer)();

    /**
     * @brief Gets colored byte buffer for visualization
     * @details
     * Generates a buffer of RGBA color values for visualization purposes.
     * Each cell in the occupancy map is assigned a color based on its state.
     *
     * @param[in] occupied Color for occupied cells as RGBA integers (0-255)
     * @param[in] unoccupied Color for unoccupied cells as RGBA integers (0-255)
     * @param[in] unknown Color for unknown cells as RGBA integers (0-255)
     *
     * @return Vector of char values representing RGBA colors for each cell in the map
     */
    std::vector<char>(CARB_ABI* getColoredByteBuffer)(const carb::Int4& occupied,
                                                      const carb::Int4& unoccupied,
                                                      const carb::Int4& unknown);
};

} // namespace omap
} // namespace gen
} // namespace asset
} // namespace isaacsim
