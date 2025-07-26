// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/asset/gen/omap/IOccupancyMap.h>
#include <isaacsim/asset/gen/omap/MapGenerator.h>
#include <omni/physx/IPhysx.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


CARB_BINDINGS("isaacsim.asset.gen.omap.python")

namespace isaacsim
{
namespace asset
{
namespace gen
{
namespace omap
{
}
}
}
}

namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_omap, m)
{
    using namespace carb;
    using namespace isaacsim::asset::gen::omap;
    // We use carb data types, must import bindings for them
    auto carbModule = py::module::import("carb");

    m.doc() = R"doc(Isaac Sim Occupancy Map Generator.

This module provides functionality for generating 2D and 3D occupancy maps from USD stages.
It allows for spatial representation of environments for robotics applications such as
navigation, path planning, and collision avoidance.
)doc";


    py::class_<MapGenerator>(m, "Generator", R"doc(Generator for creating occupancy maps from USD stages.

This class is used to generate an occupancy map for a USD stage.
Assuming the stage has collision geometry information, it can produce
2D or 3D representations of the environment's occupied and free space.

Attributes:
    None

Example:

    .. code-block:: python

        >>> import omni
        >>> from isaacsim.asset.gen.omap.bindings import _omap
        >>> 
        >>> physx = omni.physx.get_physx_interface()
        >>> stage_id = omni.usd.get_context().get_stage_id()
        >>> 
        >>> generator = _omap.Generator(physx, stage_id)
        >>> # 0.05m cell size, output buffer will have 4 for occupied cells, 
        >>> # 5 for unoccupied, and 6 for cells that cannot be seen
        >>> # this assumes your usd stage units are in m, and not cm
        >>> generator.update_settings(.05, 4, 5, 6)
        >>> # Set location to map from and the min and max bounds to map to
        >>> generator.set_transform((0, 0, 0), (-2, -2, 0), (2, 2, 0))
        >>> generator.generate2d()
        >>> # Get locations of the occupied cells in the stage
        >>> points = generator.get_occupied_positions()
        >>> # Get computed 2d occupancy buffer
        >>> buffer = generator.get_buffer()
        >>> # Get dimensions for 2d buffer
        >>> dims = generator.get_dimensions()
)doc")
        .def(py::init(
                 [](omni::physx::IPhysx* physXPtr, const long int stageId)
                 {
                     pxr::UsdStageWeakPtr stage =
                         pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
                     return new MapGenerator(physXPtr, stage);
                 }),
             R"doc(Initialize a new Generator instance.

Args:
    physx_ptr: Pointer to PhysX interface used for collision detection.
    stage_id: Stage ID for the USD stage to map.

Returns:
    A new Generator instance.
)doc")
        .def("update_settings", &MapGenerator::updateSettings,
             R"doc(Update the settings used for generating the occupancy map.

Args:
    cell_size (float): Size of the cell in stage units, resolution of the grid.
    occupied_value (float): Value used to denote an occupied cell.
    unoccupied_value (float): Value used to denote an unoccupied cell.
    unknown_value (float): Value used to denote unknown areas that could not be reached from the starting location.

Returns:
    None
)doc")
        .def("set_transform", &MapGenerator::setTransform, R"doc(Set origin and bounds for mapping.

Args:
    origin (tuple of float): Origin in stage to start mapping from, must be in unoccupied space.
    min_bound (tuple of float): Minimum bound to map up to.
    max_bound (tuple of float): Maximum bound to map up to.

Returns:
    None
)doc")
        .def("generate2d", &MapGenerator::generate2d, R"doc(Generate a 2D occupancy map.

Generates a map based on the settings and transform set. Assumes that 
a 2D map is generated and flattens the computed data.

Args:
    None

Returns:
    None
)doc")
        .def("generate3d", &MapGenerator::generate3d, R"doc(Generate a 3D occupancy map.

Generates a map based on the settings and transform set. Creates a full
3D volumetric representation of the scene's occupancy.

Args:
    None

Returns:
    None
)doc")
        .def("get_occupied_positions", &MapGenerator::getOccupiedPositions, R"doc(Get positions of all occupied cells.

Returns:
    list: List of 3D points in stage coordinates from the generated map, 
          containing occupied locations.
)doc")
        .def("get_free_positions", &MapGenerator::getFreePositions, R"doc(Get positions of all free (unoccupied) cells.

Returns:
    list: List of 3D points in stage coordinates from the generated map, 
          containing free locations.
)doc")
        .def("get_min_bound", &MapGenerator::getMinBound, R"doc(Get the minimum boundary point of the map.

Returns:
    tuple: Minimum bound for generated occupancy map in stage coordinates.
)doc")
        .def("get_max_bound", &MapGenerator::getMaxBound, R"doc(Get the maximum boundary point of the map.

Returns:
    tuple: Maximum bound for generated occupancy map in stage coordinates.
)doc")
        .def("get_dimensions", &MapGenerator::getDimensions, R"doc(Get the dimensions of the map in cell units.

Returns:
    tuple: Dimensions for output buffer (width, height, depth).
)doc")
        .def("get_buffer", &MapGenerator::getBuffer, R"doc(Get the raw occupancy buffer.

Returns:
    list: 2D array containing values for each cell in the occupancy map.
        Values correspond to the occupancy state of each cell (occupied,
        unoccupied, or unknown) as configured in update_settings().
)doc")
        .def("get_colored_byte_buffer", &MapGenerator::getColoredByteBuffer, R"doc(Generate a colored visualization buffer.

Creates an RGBA buffer for visualization purposes, where each cell
is colored according to its occupancy state.

Args:
    occupied_color (tuple): RGBA Value used to denote an occupied cell.
    unoccupied_color (tuple): RGBA Value used to denote an unoccupied cell.
    unknown_color (tuple): RGBA Value used to denote unknown areas that could not be reached.

Returns:
    list: Flattened buffer containing list of RGBA values for each pixel. 
          Can be used to render as image directly.
)doc");


    defineInterfaceClass<OccupancyMap>(m, "OccupancyMap", "acquire_omap_interface", "release_omap_interface")

        .def("generate", wrapInterfaceFunction(&OccupancyMap::generateMap), R"doc(Generate the occupancy map.

Initializes and creates an occupancy map based on the current scene.
This should be called after setting the transform and cell size parameters.

Args:
    None

Returns:
    None
)doc")
        .def("update", wrapInterfaceFunction(&OccupancyMap::update), R"doc(Update the visualization.

Refreshes the visual representation of the occupancy map.
This includes drawing the bounding box, grid, and coordinate axes.

Args:
    None

Returns:
    None
)doc")
        .def("set_transform", wrapInterfaceFunction(&OccupancyMap::setTransform),
             R"doc(Set the transform for map generation.

Defines the origin and boundaries for the occupancy map in world coordinates.

Args:
    origin (tuple): Origin point in world coordinates, serves as the reference point.
    min_point (tuple): Minimum bounds relative to origin, defines the lower corner of the map volume.
    max_point (tuple): Maximum bounds relative to origin, defines the upper corner of the map volume.

Returns:
    None
)doc")
        .def("set_cell_size", wrapInterfaceFunction(&OccupancyMap::setCellSize), R"doc(Set the cell size for the map.

Configures the resolution of the occupancy map by setting the size of each cell.

Args:
    cell_size (float): Size of each cell in meters.

Returns:
    None
)doc")
        .def("get_occupied_positions", wrapInterfaceFunction(&OccupancyMap::getOccupiedPositions),
             R"doc(Get positions of occupied cells.

Retrieves the world positions of all cells that are marked as occupied.

Args:
    None

Returns:
    list: List of 3D positions representing the centers of occupied cells.
)doc")
        .def("get_free_positions", wrapInterfaceFunction(&OccupancyMap::getFreePositions),
             R"doc(Get positions of free cells.

Retrieves the world positions of all cells that are marked as free (unoccupied).

Args:
    None

Returns:
    list: List of 3D positions representing the centers of free cells.
)doc")
        .def("get_min_bound", wrapInterfaceFunction(&OccupancyMap::getMinBound), R"doc(Get minimum bounds of the map.

Retrieves the minimum coordinate values of the occupancy map's bounding box.

Args:
    None

Returns:
    tuple: Minimum bounds as (x, y, z) in world coordinates.
)doc")
        .def("get_max_bound", wrapInterfaceFunction(&OccupancyMap::getMaxBound), R"doc(Get maximum bounds of the map.

Retrieves the maximum coordinate values of the occupancy map's bounding box.

Args:
    None

Returns:
    tuple: Maximum bounds as (x, y, z) in world coordinates.
)doc")
        .def("get_dimensions", wrapInterfaceFunction(&OccupancyMap::getDimensions),
             R"doc(Get dimensions of the map in cells.

Retrieves the number of cells along each axis of the occupancy map.

Args:
    None

Returns:
    tuple: Dimensions as (width, height, depth), where each component represents 
           the number of cells along that axis.
)doc")
        .def("get_buffer", wrapInterfaceFunction(&OccupancyMap::getBuffer), R"doc(Get the occupancy buffer.

Retrieves the raw occupancy values for all cells in the map.

Args:
    None

Returns:
    list: Vector of cell values, where each value indicates the occupancy state
         (typically 1.0 for occupied, 0.0 for free, and 0.5 for unknown).
)doc")
        .def("get_colored_byte_buffer", wrapInterfaceFunction(&OccupancyMap::getColoredByteBuffer),
             R"doc(Get colored byte buffer for visualization.

Generates a buffer of RGBA color values for visualization purposes.
Each cell in the occupancy map is assigned a color based on its state.

Args:
    occupied (tuple): Color for occupied cells as RGBA integers (0-255).
    unoccupied (tuple): Color for unoccupied cells as RGBA integers (0-255).
    unknown (tuple): Color for unknown cells as RGBA integers (0-255).

Returns:
    list: Vector of byte values representing RGBA colors for each cell in the map.
)doc");
}
}
