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

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/asset/gen/omap/IOccupancyMap.h>
#include <isaacsim/asset/gen/omap/MapGenerator.h>
#include <isaacsim/util/debug_draw/PrimitiveDrawingHelper.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>

#include <map>
#include <string>
#include <vector>

namespace
{
const carb::PluginImplDesc g_kPluginDesc = { "isaacsim.asset.gen.omap.plugin", "Isaac Motion Planning", "NVIDIA",
                                             carb::PluginHotReload::eDisabled, "dev" };

pxr::UsdStageWeakPtr g_stage = nullptr;
omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
omni::physx::IPhysx* g_physx = nullptr;
carb::Float3 g_inputOrigin = { 0.0f, 0.0f, 0.0f };
carb::Float3 g_inputMinPoint = { -1.0f, -1.0f, 0.0f };
carb::Float3 g_inputMaxPoint = { 1.0f, 1.0f, 0.0f };
float g_inputCellSize = 0.05f;
float g_metersPerUnit = 1.0f;

std::unique_ptr<isaacsim::asset::gen::omap::MapGenerator> g_generator = nullptr;
std::unique_ptr<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper> g_lineDrawing;
std::unique_ptr<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper> g_cellDrawing;
}

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::asset::gen::omap::OccupancyMap)
CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx, omni::kit::IStageUpdate)

/**
 * @brief Generates a new occupancy map
 * @details
 * Creates a new MapGenerator instance using the current PhysX interface and USD stage.
 * Configures the generator with the current transform and cell size settings,
 * then generates a 2D occupancy map.
 *
 * @pre g_physx and g_stage must be valid
 * @post g_generator will be initialized with a new MapGenerator instance
 */
void CARB_ABI generateMap()
{

    g_generator = std::make_unique<isaacsim::asset::gen::omap::MapGenerator>(g_physx, g_stage);

    g_generator->setTransform(g_inputOrigin, g_inputMinPoint, g_inputMaxPoint);
    g_generator->updateSettings(g_inputCellSize, 1.0f, 0.0f, 0.5f);
    g_generator->generate2d();
}

/**
 * @brief Sets the transform parameters for map generation
 * @details
 * Updates the global transform variables used for map generation.
 * These values define the origin and extent of the area to be mapped.
 *
 * @param[in] origin The origin point in world coordinates
 * @param[in] minimum The minimum point relative to the origin
 * @param[in] maximum The maximum point relative to the origin
 *
 * @post g_inputOrigin, g_inputMinPoint, and g_inputMaxPoint will be updated
 */
void CARB_ABI setTransform(carb::Float3 origin, carb::Float3 minimum, carb::Float3 maximum)
{
    g_inputOrigin = origin;
    g_inputMinPoint = minimum;
    g_inputMaxPoint = maximum;
}

/**
 * @brief Sets the cell size for map generation
 * @details
 * Updates the global cell size variable used for map generation.
 * Cell size determines the resolution of the generated occupancy map.
 *
 * @param[in] cellSize Size of each cell in meters
 *
 * @pre cellSize should be positive
 * @post g_inputCellSize will be updated with the provided value or a default value if invalid
 *
 * @note If cellSize is less than or equal to 0, a default value of 0.01 meters will be used
 */
void CARB_ABI setCellSize(float cellSize)
{
    if (cellSize <= 0)
    {
        cellSize = .01f / g_metersPerUnit;
        CARB_LOG_WARN("Cell size is less than or equal to 0. A value of 0.01 meters will be used instead.");
    }
    g_inputCellSize = cellSize;
}

/**
 * @brief Helper function to draw a bounding box
 * @details
 * Draws lines connecting the corners of a bounding box with the specified color and line width.
 *
 * @param[in] corners Vector of 8 points representing the corners of the bounding box
 * @param[in] color RGBA color to use for the lines
 * @param[in] lineWidth Width of the lines in pixels
 *
 * @pre g_lineDrawing must be initialized
 * @pre corners must contain exactly 8 points representing the box corners
 */
void drawBoundingBox(const std::vector<carb::Float3>& corners, const carb::ColorRgba& color, float lineWidth)
{
    // Bottom face
    for (int i = 0; i < 4; i++)
    {
        g_lineDrawing->addVertex(corners[i], color, lineWidth);
        g_lineDrawing->addVertex(corners[(i + 1) % 4], color, lineWidth);
    }

    // Top face
    for (int i = 4; i < 8; i++)
    {
        g_lineDrawing->addVertex(corners[i], color, lineWidth);
        g_lineDrawing->addVertex(corners[(i + 1) % 4 + 4], color, lineWidth);
    }

    // Vertical edges
    for (int i = 0; i < 4; i++)
    {
        g_lineDrawing->addVertex(corners[i], color, lineWidth);
        g_lineDrawing->addVertex(corners[i + 4], color, lineWidth);
    }
}

/**
 * @brief Helper function to draw a grid
 * @details
 * Draws a grid of horizontal and vertical lines within the defined map boundaries.
 * The spacing between grid lines is determined by the cell size.
 *
 * @param[in] lineWidth Width of the grid lines in pixels
 *
 * @pre g_lineDrawing must be initialized
 * @pre g_inputOrigin, g_inputMinPoint, g_inputMaxPoint, and g_inputCellSize must be valid
 */
void drawGrid(float lineWidth)
{
    carb::ColorRgba gridColor = { 0.5f, 0.5f, 0.5f, 0.5f };

    // Draw vertical grid lines
    for (float ix = g_inputMinPoint.x; ix <= g_inputMaxPoint.x; ix += g_inputCellSize)
    {
        carb::Float3 p0{ g_inputOrigin.x + ix, g_inputOrigin.y + g_inputMinPoint.y, g_inputOrigin.z };
        carb::Float3 p1{ g_inputOrigin.x + ix, g_inputOrigin.y + g_inputMaxPoint.y, g_inputOrigin.z };

        g_lineDrawing->addVertex(p0, gridColor, lineWidth);
        g_lineDrawing->addVertex(p1, gridColor, lineWidth);
    }

    // Draw horizontal grid lines
    for (float iy = g_inputMinPoint.y; iy <= g_inputMaxPoint.y; iy += g_inputCellSize)
    {
        carb::Float3 p0{ g_inputOrigin.x + g_inputMinPoint.x, g_inputOrigin.y + iy, g_inputOrigin.z };
        carb::Float3 p1{ g_inputOrigin.x + g_inputMaxPoint.x, g_inputOrigin.y + iy, g_inputOrigin.z };

        g_lineDrawing->addVertex(p0, gridColor, lineWidth);
        g_lineDrawing->addVertex(p1, gridColor, lineWidth);
    }
}

/**
 * @brief Updates the visualization of the occupancy map
 * @details
 * Clears previous visualization elements and redraws the bounding box,
 * grid, and coordinate axes based on the current settings.
 *
 * @pre g_lineDrawing and g_cellDrawing must be initialized
 * @post The visualization will be updated to reflect the current map settings
 */
void CARB_ABI update()
{
    g_lineDrawing->clear();
    g_cellDrawing->clear();
    float lineWidth = 2.0f;

    // Calculate corners
    std::vector<carb::Float3> corners(8);
    corners[0] = carb::Float3({ g_inputOrigin.x + g_inputMinPoint.x, g_inputOrigin.y + g_inputMinPoint.y,
                                g_inputOrigin.z + g_inputMinPoint.z });
    corners[1] = carb::Float3({ g_inputOrigin.x + g_inputMaxPoint.x, g_inputOrigin.y + g_inputMinPoint.y,
                                g_inputOrigin.z + g_inputMinPoint.z });
    corners[2] = carb::Float3({ g_inputOrigin.x + g_inputMaxPoint.x, g_inputOrigin.y + g_inputMaxPoint.y,
                                g_inputOrigin.z + g_inputMinPoint.z });
    corners[3] = carb::Float3({ g_inputOrigin.x + g_inputMinPoint.x, g_inputOrigin.y + g_inputMaxPoint.y,
                                g_inputOrigin.z + g_inputMinPoint.z });
    corners[4] = carb::Float3({ g_inputOrigin.x + g_inputMinPoint.x, g_inputOrigin.y + g_inputMinPoint.y,
                                g_inputOrigin.z + g_inputMaxPoint.z });
    corners[5] = carb::Float3({ g_inputOrigin.x + g_inputMaxPoint.x, g_inputOrigin.y + g_inputMinPoint.y,
                                g_inputOrigin.z + g_inputMaxPoint.z });
    corners[6] = carb::Float3({ g_inputOrigin.x + g_inputMaxPoint.x, g_inputOrigin.y + g_inputMaxPoint.y,
                                g_inputOrigin.z + g_inputMaxPoint.z });
    corners[7] = carb::Float3({ g_inputOrigin.x + g_inputMinPoint.x, g_inputOrigin.y + g_inputMaxPoint.y,
                                g_inputOrigin.z + g_inputMaxPoint.z });

    // Draw bounding box
    drawBoundingBox(corners, { 1, 1, 1, 1 }, lineWidth);

    // Draw grid
    drawGrid(lineWidth);

    // Draw coordinate axes
    carb::Float3 scaleMin = g_inputMinPoint;
    carb::Float3 scaleMax = g_inputMaxPoint;

    // Ensure minimum axis size
    float minSize = 0.1f / g_metersPerUnit;

    // Check and adjust X dimension
    if (scaleMax.x - scaleMin.x < minSize)
    {
        scaleMin.x -= minSize;
        scaleMax.x += minSize;
    }

    // Check and adjust Y dimension
    if (scaleMax.y - scaleMin.y < minSize)
    {
        scaleMin.y -= minSize;
        scaleMax.y += minSize;
    }

    // Check and adjust Z dimension
    if (scaleMax.z - scaleMin.z < minSize)
    {
        scaleMin.z -= minSize;
        scaleMax.z += minSize;
    }

    // Draw X axis (red)
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x + scaleMin.x, g_inputOrigin.y, g_inputOrigin.z }),
                             { 1, 0, 0, 1 }, lineWidth * 2);
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x + scaleMax.x, g_inputOrigin.y, g_inputOrigin.z }),
                             { 1, 0, 0, 1 }, lineWidth * 2);

    // Draw Y axis (green)
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x, g_inputOrigin.y + scaleMin.y, g_inputOrigin.z }),
                             { 0, 1, 0, 1 }, lineWidth * 2);
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x, g_inputOrigin.y + scaleMax.y, g_inputOrigin.z }),
                             { 0, 1, 0, 1 }, lineWidth * 2);

    // Draw Z axis (blue)
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x, g_inputOrigin.y, g_inputOrigin.z + scaleMin.z }),
                             { 0, 0, 1, 1 }, lineWidth * 2);
    g_lineDrawing->addVertex(carb::Float3({ g_inputOrigin.x, g_inputOrigin.y, g_inputOrigin.z + scaleMax.z }),
                             { 0, 0, 1, 1 }, lineWidth * 2);

    g_lineDrawing->draw();
}

/**
 * @brief Retrieves positions of all occupied cells
 * @details
 * Gets the 3D positions of all cells that are marked as occupied in the current map.
 *
 * @return Vector of 3D positions representing the centers of occupied cells
 *
 * @note Returns an empty vector if g_generator is not initialized
 */
std::vector<carb::Float3> getOccupiedPositions()
{
    std::vector<carb::Float3> pos;
    if (g_generator)
    {
        pos = g_generator->getOccupiedPositions();
    }
    return pos;
}

/**
 * @brief Retrieves positions of all free cells
 * @details
 * Gets the 3D positions of all cells that are marked as free (unoccupied) in the current map.
 *
 * @return Vector of 3D positions representing the centers of free cells
 *
 * @note Returns an empty vector if g_generator is not initialized
 */
std::vector<carb::Float3> getFreePositions()
{
    std::vector<carb::Float3> pos;
    if (g_generator)
    {
        pos = g_generator->getFreePositions();
    }
    return pos;
}

/**
 * @brief Gets the minimum boundary point of the map
 * @details
 * Retrieves the coordinates of the minimum boundary point of the occupancy map.
 *
 * @return Minimum boundary coordinates as Float3
 *
 * @note Returns (0,0,0) if g_generator is not initialized
 */
carb::Float3 getMinBound()
{
    carb::Float3 bounds = { 0, 0, 0 };
    if (g_generator)
    {
        bounds = g_generator->getMinBound();
    }
    return bounds;
}

/**
 * @brief Gets the maximum boundary point of the map
 * @details
 * Retrieves the coordinates of the maximum boundary point of the occupancy map.
 *
 * @return Maximum boundary coordinates as Float3
 *
 * @note Returns (0,0,0) if g_generator is not initialized
 */
carb::Float3 getMaxBound()
{
    carb::Float3 bounds = { 0, 0, 0 };
    if (g_generator)
    {
        bounds = g_generator->getMaxBound();
    }
    return bounds;
}

/**
 * @brief Gets the dimensions of the map in cell units
 * @details
 * Retrieves the number of cells along each dimension of the occupancy map.
 *
 * @return Number of cells in each dimension as Int3
 *
 * @note Returns (0,0,0) if g_generator is not initialized
 */
carb::Int3 getDimensions()
{
    carb::Int3 dims = { 0, 0, 0 };
    if (g_generator)
    {
        dims = g_generator->getDimensions();
    }
    return dims;
}

/**
 * @brief Retrieves the raw occupancy buffer
 * @details
 * Gets the raw occupancy values for all cells in the current map.
 *
 * @return Vector of occupancy values for all cells
 *
 * @note Returns an empty vector if g_generator is not initialized
 */
std::vector<float> getBuffer()
{
    if (g_generator)
    {
        return g_generator->getBuffer();
    }
    return std::vector<float>();
}

/**
 * @brief Generates a colored visualization buffer
 * @details
 * Creates a buffer of RGBA color values for visualization purposes,
 * with distinct colors for occupied, unoccupied, and unknown cells.
 *
 * @param[in] occupied Color for occupied cells (RGBA)
 * @param[in] unoccupied Color for unoccupied cells (RGBA)
 * @param[in] unknown Color for unknown cells (RGBA)
 *
 * @return Vector of bytes representing RGBA values for each cell
 *
 * @note Returns an empty vector if g_generator is not initialized
 */
std::vector<char> getColoredByteBuffer(const carb::Int4& occupied, const carb::Int4& unoccupied, const carb::Int4& unknown)
{
    if (g_generator)
    {
        return g_generator->getColoredByteBuffer(occupied, unoccupied, unknown);
    }
    return std::vector<char>();
}

/**
 * @brief Callback function when a stage is attached
 * @details
 * Initializes the USD stage and drawing helpers when a stage is attached.
 *
 * @param[in] stageId ID of the attached USD stage
 * @param[in] metersPerUnit Conversion factor from scene units to meters
 * @param[in] userData User data pointer
 *
 * @pre stageId must correspond to a valid USD stage
 * @post g_stage, g_metersPerUnit, g_lineDrawing, and g_cellDrawing will be initialized
 */
static void onAttach(long int stageId, double metersPerUnit, void* userData)
{
    // try and find USD stage from Id
    pxr::UsdStageWeakPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

    if (!stage)
    {
        CARB_LOG_ERROR("Isaac OccupancyMap could not find USD stage");
        return;
    }

    g_stage = stage;
    g_metersPerUnit = static_cast<float>(metersPerUnit);
    g_lineDrawing = std::make_unique<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper>(
        omni::usd::UsdContext::getContext(),
        isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper::RenderingMode::eLines);

    g_cellDrawing = std::make_unique<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper>(
        omni::usd::UsdContext::getContext(),
        isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper::RenderingMode::eLines, true);
}

/**
 * @brief Callback function when a stage is detached
 * @details
 * Cleans up resources when a stage is detached.
 *
 * @param[in] data User data pointer
 *
 * @post g_lineDrawing and g_cellDrawing will be reset
 */
static void onDetach(void* data)
{

    g_lineDrawing.reset();
    g_cellDrawing.reset();
}

/**
 * @brief Update callback for the stage
 * @details
 * Called regularly during stage updates to perform ongoing processing.
 * Currently only performs operations when the stage is playing.
 *
 * @param[in] currentTime Current simulation time
 * @param[in] elapsedSecs Time elapsed since the last update
 * @param[in] settings Stage update settings
 * @param[in] userData User data pointer
 */
void onUpdate(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings* settings, void* userData)
{

    if (!settings->isPlaying)
    {
        return;
    }
}

/**
 * @brief Plugin startup function
 * @details
 * Initializes plugin resources when the plugin is loaded.
 * Sets up the stage update node and acquires required interfaces.
 *
 * @pre PhysX interface must be available
 * @post g_stageUpdate, g_physx, and g_stageUpdateNode will be initialized
 */
CARB_EXPORT void carbOnPluginStartup()
{
    g_stageUpdate = carb::getCachedInterface<omni::kit::IStageUpdate>()->getStageUpdate();

    g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
    if (!g_physx)
    {
        CARB_LOG_ERROR("*** Failed to acquire PhysX interface\n");
        return;
    }

    omni::kit::StageUpdateNodeDesc desc = { nullptr };
    desc.displayName = "OccupancyMap";
    desc.onAttach = onAttach;
    desc.onDetach = onDetach;
    // Create the stage update node and make sure it runs after physx
    size_t index = g_stageUpdate->getStageUpdateNodeCount();
    g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);
    g_stageUpdate->setStageUpdateNodeOrder(index, 75);
}

/**
 * @brief Plugin shutdown function
 * @details
 * Cleans up plugin resources when the plugin is unloaded.
 *
 * @post g_stageUpdateNode will be destroyed, g_lineDrawing and g_cellDrawing will be reset
 */
CARB_EXPORT void carbOnPluginShutdown()
{
    g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
    g_lineDrawing.reset();
    g_cellDrawing.reset();
}

/**
 * @brief Fills the OccupancyMap interface
 * @details
 * Populates the OccupancyMap interface structure with function pointers
 * to the implementation functions.
 *
 * @param[out] iface Reference to the OccupancyMap interface to be filled
 *
 * @post All interface function pointers will be initialized
 */
void fillInterface(isaacsim::asset::gen::omap::OccupancyMap& iface)
{
    iface.generateMap = generateMap;
    iface.update = update;
    iface.setTransform = setTransform;
    iface.setCellSize = setCellSize;
    iface.getOccupiedPositions = getOccupiedPositions;
    iface.getFreePositions = getFreePositions;
    iface.getMinBound = getMinBound;
    iface.getMaxBound = getMaxBound;
    iface.getDimensions = getDimensions;
    iface.getBuffer = getBuffer;
    iface.getColoredByteBuffer = getColoredByteBuffer;
}
