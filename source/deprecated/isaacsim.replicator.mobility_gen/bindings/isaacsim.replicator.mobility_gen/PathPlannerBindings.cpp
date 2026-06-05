// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

namespace py = pybind11;


struct Point
{
    int64_t row;
    int64_t column;

    bool operator==(const Point& other) const
    {
        return (row == other.row) && (column == other.column);
    }
};


struct PriorityQueueItem
{
    double priority;
    Point point;
};


struct PriorityQueueCompare
{
    bool operator()(const PriorityQueueItem& a, const PriorityQueueItem& b) const
    {
        return a.priority > b.priority;
    }
};


float getDistance(Point& start, Point& end)
{
    float rowOffset = (float)(end.row - start.row);
    float columnOffset = (float)(end.column - start.column);
    return std::sqrt(rowOffset * rowOffset + columnOffset * columnOffset);
}


std::vector<Point> getChildren(Point& point,
                               py::detail::unchecked_reference<uint8_t, 2> freespaceMap,
                               py::detail::unchecked_mutable_reference<uint8_t, 2> visitedMap,
                               int64_t rowCount,
                               int64_t columnCount)
{

    // Initialize candidate locations
    std::vector<Point> candidates = { // Top row
                                      { point.row - 1, point.column - 1 },
                                      { point.row - 1, point.column },
                                      { point.row - 1, point.column + 1 },

                                      // Middle row (exclude self)
                                      { point.row, point.column - 1 },
                                      { point.row, point.column + 1 },

                                      // Bottom row
                                      { point.row + 1, point.column - 1 },
                                      { point.row + 1, point.column },
                                      { point.row + 1, point.column + 1 }
    };

    // Initialize output children
    std::vector<Point> children;
    children.reserve(8);

    // Filter candidates and populate output
    for (int i = 0; i < 8; i++)
    {

        auto candidate = candidates[i];

        // Filter out of bounds
        if (candidate.row < 0)
        {
            continue;
        }
        if (candidate.row >= rowCount)
        {
            continue;
        }
        if (candidate.column < 0)
        {
            continue;
        }
        if (candidate.column >= columnCount)
        {
            continue;
        }

        // Filter non-freespaceMap
        if (!freespaceMap(candidate.row, candidate.column))
        {
            continue;
        }

        // Filter visitedMap
        if (visitedMap(candidate.row, candidate.column))
        {
            continue;
        }

        children.push_back(candidate);
    }

    return children;
}

void generatePaths(py::array_t<int64_t> startPoint,
                   py::array_t<uint8_t> freespaceMap,
                   py::array_t<uint8_t> visitedMap,
                   py::array_t<double> distanceToStartMap,
                   py::array_t<int64_t> parentRowMap,
                   py::array_t<int64_t> parentColumnMap)
{

    size_t rowCount = freespaceMap.shape(0);
    size_t columnCount = freespaceMap.shape(1);
    auto startPointUnchecked = startPoint.unchecked<1>();

    Point start = { startPointUnchecked(0), startPointUnchecked(1) };

    auto freespaceMapUnchecked = freespaceMap.unchecked<2>();
    auto visitedMapUnchecked = visitedMap.mutable_unchecked<2>();
    auto distanceToStartMapUnchecked = distanceToStartMap.mutable_unchecked<2>();
    auto parentRowMapUnchecked = parentRowMap.mutable_unchecked<2>();
    auto parentColumnMapUnchecked = parentColumnMap.mutable_unchecked<2>();

    std::priority_queue<PriorityQueueItem, std::vector<PriorityQueueItem>, PriorityQueueCompare> queue;

    // Initialize
    distanceToStartMapUnchecked(start.row, start.column) = 0.0;
    queue.push({ 0., start });
    visitedMapUnchecked(start.row, start.column) = true;

    // Search
    while (!queue.empty())
    {

        PriorityQueueItem node = queue.top();

        queue.pop();

        // Add children
        std::vector<Point> children =
            getChildren(node.point, freespaceMapUnchecked, visitedMapUnchecked, rowCount, columnCount);

        for (unsigned int i = 0; i < children.size(); i++)
        {

            Point child = children[i];

            // Mark parent node, for unrolling path
            parentRowMapUnchecked(child.row, child.column) = node.point.row;
            parentColumnMapUnchecked(child.row, child.column) = node.point.column;

            // Mark distance to start

            double childDistanaceToStart =
                (distanceToStartMapUnchecked(node.point.row, node.point.column) + getDistance(node.point, child));

            distanceToStartMapUnchecked(child.row, child.column) = childDistanaceToStart;

            visitedMapUnchecked(child.row, child.column) = true;

            queue.push({ childDistanaceToStart, child });
        }
    }
}

std::vector<std::pair<int64_t, int64_t>> unrollPath(py::array_t<int64_t> endPoint,
                                                    py::array_t<int64_t> parentRowMap,
                                                    py::array_t<int64_t> parentColumnMap)
{

    auto parentRowMapUnchecked = parentRowMap.unchecked<2>();
    auto parentColumnMapUnchecked = parentColumnMap.unchecked<2>();

    auto endPointUnchecked = endPoint.unchecked<1>();

    std::vector<std::pair<int64_t, int64_t>> path;
    std::pair<int64_t, int64_t> point = { endPointUnchecked(0), endPointUnchecked(1) };

    while (true)
    {
        path.push_back(point);

        if (parentRowMapUnchecked(point.first, point.second) < 0)
        {
            break; // end of path
        }

        point = { parentRowMapUnchecked(point.first, point.second), parentColumnMapUnchecked(point.first, point.second) };
    }

    std::reverse(path.begin(), path.end());

    return path;
}

PYBIND11_MODULE(_path_planner, m)
{
    m.doc() = "MobilityGen Path Planner C++ Bindings";
    m.def("generate_paths", &generatePaths, "Generate paths");
    m.def("unroll_path", &unrollPath, "Unroll a path");
}
