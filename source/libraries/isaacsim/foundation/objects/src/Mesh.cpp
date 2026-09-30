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

#include <isaacsim/foundation/objects/Mesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

namespace
{

// Mesh attributes are ragged (each prim holds a different number of points, faces, creases, ...), so they are
// read/written one prim at a time by wrapping a single path with a Prim rather than batching over all of them.
// Values keep the leading batch dimension expected by the attribute accessors: reads have shape (1, ...) and are
// unwrapped with 'at(0)', writes are reshaped to (1, -1) or (1, -1, 3).

// Select the item to apply to the i-th selected prim, broadcasting a single-item list to all of them.
const array::Array& resolveItem(const std::vector<array::Array>& items,
                                std::size_t index,
                                std::size_t batchSize,
                                const char* name)
{
    if (items.size() == 1)
    {
        return items.front();
    }
    if (items.size() != batchSize)
    {
        throw std::invalid_argument("Expected a list of arrays with length 1 or " + std::to_string(batchSize) +
                                    " for '" + name + "', got a list with length " + std::to_string(items.size()));
    }
    return items[index];
}

// The primitive generators below mirror the evaluators of the 'omni.kit.primitive.mesh' Kit extension, which is the
// implementation the Python 'Mesh' class reaches through the 'CreateMeshPrimWithDefaultXformCommand' command.
// They only emit Z-up geometry: the winding order and texture coordinates of the Y-up variants differ, and Isaac Sim
// stages are Z-up.

constexpr double g_kPi = 3.14159265358979323846;

using Vector2 = std::array<double, 2>;
using Vector3 = std::array<double, 3>;

// Face-varying geometry of a generated primitive. 'points' is XYZ-interleaved and 'textureCoordinates' is
// UV-interleaved. 'points' holds one entry per mesh vertex, while 'normals' and 'textureCoordinates' hold one entry
// per *face* vertex, that is, one per element of 'faceVertexIndices'.
struct MeshData
{
    std::vector<float> points;
    std::vector<float> normals;
    std::vector<float> textureCoordinates;
    std::vector<int64_t> faceVertexIndices;
    std::vector<int64_t> faceVertexCounts;
};

// Which pair of axes a quad grid spans, named after the axis it faces along.
enum class GridAxis
{
    eX,
    eY,
    eZ
};

// Geometry is computed in double precision and only narrowed to float when it is appended, since USD stores mesh
// points, normals and texture coordinates as single precision.
void appendVector2(std::vector<float>& values, const Vector2& value)
{
    values.push_back(static_cast<float>(value[0]));
    values.push_back(static_cast<float>(value[1]));
}

void appendVector3(std::vector<float>& values, const Vector3& value)
{
    values.push_back(static_cast<float>(value[0]));
    values.push_back(static_cast<float>(value[1]));
    values.push_back(static_cast<float>(value[2]));
}

Vector3 readVector3(const std::vector<float>& values, int64_t index)
{
    const std::size_t offset = 3 * static_cast<std::size_t>(index);
    return { values[offset], values[offset + 1], values[offset + 2] };
}

Vector3 scaleAndOffsetPoint(const Vector3& point, const Vector3& origin, double scale)
{
    return { scale * point[0] + origin[0], scale * point[1] + origin[1], scale * point[2] + origin[2] };
}

// Scale a vector to unit length, leaving a zero-length vector untouched.
Vector3 normalized(const Vector3& value)
{
    const double length = std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
    if (length == 0.0)
    {
        return value;
    }
    return { value[0] / length, value[1] / length, value[2] / length };
}

Vector3 normalizedCrossProduct(const Vector3& first, const Vector3& second)
{
    return normalized({ first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2],
                        first[0] * second[1] - first[1] * second[0] });
}

// Apply the tessellation multiplier to a patch count and clamp it to the minimum the topology requires.
int64_t resolvePatchCount(int64_t patches, int64_t verticesScale, int64_t minimum)
{
    return std::max(patches * std::max(verticesScale, int64_t{ 1 }), minimum);
}

// Reverse the orientation of every face by reversing all of its vertices but the first. 'values' is a flat buffer of
// 'stride' components per face vertex, laid out face after face in the order given by 'faceVertexCounts'.
template <typename T>
void modifyWindingOrder(const std::vector<int64_t>& faceVertexCounts, std::vector<T>& values, std::size_t stride)
{
    std::size_t offset = 0;
    for (int64_t count : faceVertexCounts)
    {
        if (count >= 3)
        {
            std::size_t low = offset + 1;
            std::size_t high = offset + static_cast<std::size_t>(count) - 1;
            for (; low < high; ++low, --high)
            {
                for (std::size_t component = 0; component < stride; ++component)
                {
                    std::swap(values[low * stride + component], values[high * stride + component]);
                }
            }
        }
        offset += static_cast<std::size_t>(count);
    }
}

// Points of a unit circle on the XY plane, in the order the disk cap and the cylinder body both rely on.
std::vector<Vector3> generateCirclePoints(int64_t pointCount, double delta)
{
    std::vector<Vector3> points;
    points.reserve(static_cast<std::size_t>(pointCount));
    for (int64_t i = 0; i < pointCount; ++i)
    {
        const double theta = static_cast<double>(i) * delta * 2.0 * g_kPi;
        points.push_back({ std::cos(theta), std::sin(theta), 0.0 });
    }
    return points;
}

// Disk centered on 'centerPoint', tessellated as 'uPatches' segments around the rim and 'vPatches' concentric rings.
// The rim points come first so that a caller can share them with an adjoining body, and the center point comes last.
MeshData buildDisk(const Vector3& centerPoint, int64_t uPatches, int64_t vPatches, double halfScale)
{
    const double vDelta = 1.0 / static_cast<double>(vPatches);
    const int64_t uVertexCount = uPatches;
    const int64_t vVertexCount = vPatches + 1;

    MeshData data;
    const Vector3 center = scaleAndOffsetPoint(centerPoint, { 0.0, 0.0, 0.0 }, halfScale);
    const std::vector<Vector3> circlePoints = generateCirclePoints(uPatches, 1.0 / static_cast<double>(uPatches));
    for (int64_t j = 0; j < vVertexCount - 1; ++j)
    {
        const double v = vDelta * static_cast<double>(j);
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            appendVector3(data.points,
                          scaleAndOffsetPoint(circlePoints[static_cast<std::size_t>(i)], center, halfScale * (1.0 - v)));
        }
    }
    appendVector3(data.points, center);

    // The innermost ring degenerates onto the single center point, which turns its quads into triangles.
    const auto calculateIndex = [&](int64_t i, int64_t j)
    {
        const int64_t baseIndex = j * uVertexCount;
        return j == vVertexCount - 1 ? baseIndex : baseIndex + (i < uVertexCount ? i : 0);
    };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2
    {
        // Undo the scale to bring the point back onto [-1, 1], then map that onto [0, 1].
        const Vector3 point = readVector3(data.points, calculateIndex(i, j));
        return { (point[0] / halfScale + 1.0) / 2.0, (point[1] / halfScale + 1.0) / 2.0 };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const int64_t index00 = calculateIndex(i, j);
            const int64_t index10 = calculateIndex(i + 1, j);
            const int64_t index11 = calculateIndex(i + 1, j + 1);
            const int64_t index01 = calculateIndex(i, j + 1);

            if (index11 == index01)
            {
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
                data.faceVertexIndices.insert(data.faceVertexIndices.end(), { index00, index10, index01 });
                data.faceVertexCounts.push_back(3);
            }
            else
            {
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
                data.faceVertexIndices.insert(data.faceVertexIndices.end(), { index00, index10, index11, index01 });
                data.faceVertexCounts.push_back(4);
            }
            for (int64_t vertex = 0; vertex < data.faceVertexCounts.back(); ++vertex)
            {
                appendVector3(data.normals, { 0.0, 0.0, 1.0 });
            }
        }
    }
    return data;
}

// Planar grid of 'uPatches' by 'vPatches' quads, centered on 'origin' and spanning '2 * halfScale' along both of the
// axes that 'axis' does not face along.
MeshData buildQuadGrid(const Vector3& origin, double halfScale, int64_t uPatches, int64_t vPatches, GridAxis axis)
{
    const int64_t uVertexCount = uPatches + 1;
    const int64_t vVertexCount = vPatches + 1;
    const double uDelta = 1.0 / static_cast<double>(uPatches);
    const double vDelta = 1.0 / static_cast<double>(vPatches);
    const double uStep = 2.0 * halfScale * uDelta;
    const double vStep = 2.0 * halfScale * vDelta;

    Vector3 bottomLeft{};
    Vector3 uDirection{};
    Vector3 vDirection{};
    Vector3 normal{};
    switch (axis)
    {
    case GridAxis::eX:
        bottomLeft = { origin[0], origin[1] - halfScale, origin[2] - halfScale };
        uDirection = { 0.0, 1.0, 0.0 };
        vDirection = { 0.0, 0.0, 1.0 };
        normal = { 1.0, 0.0, 0.0 };
        break;
    case GridAxis::eY:
        bottomLeft = { origin[0] - halfScale, origin[1], origin[2] - halfScale };
        uDirection = { 1.0, 0.0, 0.0 };
        vDirection = { 0.0, 0.0, 1.0 };
        normal = { 0.0, 1.0, 0.0 };
        break;
    case GridAxis::eZ:
        bottomLeft = { origin[0] - halfScale, origin[1] - halfScale, origin[2] };
        uDirection = { 1.0, 0.0, 0.0 };
        vDirection = { 0.0, 1.0, 0.0 };
        normal = { 0.0, 0.0, 1.0 };
        break;
    }

    MeshData data;
    for (int64_t j = 0; j < vVertexCount; ++j)
    {
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            const double u = static_cast<double>(i) * uStep;
            const double v = static_cast<double>(j) * vStep;
            appendVector3(data.points, { bottomLeft[0] + u * uDirection[0] + v * vDirection[0],
                                         bottomLeft[1] + u * uDirection[1] + v * vDirection[1],
                                         bottomLeft[2] + u * uDirection[2] + v * vDirection[2] });
        }
    }

    // Unlike the disk, cylinder and torus, the grid does not wrap around, so every corner has its own vertex.
    const auto calculateIndex = [&](int64_t i, int64_t j) { return j * uVertexCount + i; };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2
    {
        const double u = static_cast<double>(i) * uDelta;
        // The Y-facing grid is the only one whose V runs back towards the origin.
        if (axis == GridAxis::eY)
        {
            return { u, 1.0 - static_cast<double>(j) * vDelta };
        }
        return { u, static_cast<double>(j) * vDelta };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const int64_t index00 = calculateIndex(i, j);
            const int64_t index10 = calculateIndex(i + 1, j);
            const int64_t index11 = calculateIndex(i + 1, j + 1);
            const int64_t index01 = calculateIndex(i, j + 1);

            // The Z-facing grid winds along U first; the other two wind along V first to keep their normals outwards.
            if (axis == GridAxis::eZ)
            {
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
                data.faceVertexIndices.insert(data.faceVertexIndices.end(), { index00, index10, index11, index01 });
            }
            else
            {
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
                appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
                data.faceVertexIndices.insert(data.faceVertexIndices.end(), { index00, index01, index11, index10 });
            }
            data.faceVertexCounts.push_back(4);
            for (int64_t vertex = 0; vertex < 4; ++vertex)
            {
                appendVector3(data.normals, normal);
            }
        }
    }
    return data;
}

// Close one end of a cone or cylinder body with a disk cap. The cap reuses the body's rim vertices, which start at
// 'rimStartIndex', so only the disk's remaining points are appended.
void appendCap(MeshData& data,
               const Vector3& centerPoint,
               int64_t rimStartIndex,
               int64_t uPatches,
               int64_t wPatches,
               double halfScale,
               bool invertWindingOrder)
{
    MeshData cap = buildDisk(centerPoint, uPatches, wPatches, halfScale);
    const int64_t bodyPointCount = static_cast<int64_t>(data.points.size() / 3);

    data.points.insert(data.points.end(), cap.points.begin() + 3 * uPatches, cap.points.end());
    data.faceVertexCounts.insert(data.faceVertexCounts.end(), cap.faceVertexCounts.begin(), cap.faceVertexCounts.end());
    // The cap is flat, so every one of its face vertices shares the normal that points away from the body.
    const Vector3 capNormal = normalized(centerPoint);
    for (std::size_t i = 0; i < cap.faceVertexIndices.size(); ++i)
    {
        appendVector3(data.normals, capNormal);
    }

    for (int64_t& index : cap.faceVertexIndices)
    {
        index += index >= uPatches ? bodyPointCount - uPatches : rimStartIndex;
    }
    if (invertWindingOrder)
    {
        modifyWindingOrder(cap.faceVertexCounts, cap.faceVertexIndices, 1);
        modifyWindingOrder(cap.faceVertexCounts, cap.textureCoordinates, 2);
        for (std::size_t i = 0; i < cap.textureCoordinates.size(); i += 2)
        {
            appendVector2(data.textureCoordinates, { cap.textureCoordinates[i], 1.0 - cap.textureCoordinates[i + 1] });
        }
    }
    else
    {
        data.textureCoordinates.insert(
            data.textureCoordinates.end(), cap.textureCoordinates.begin(), cap.textureCoordinates.end());
    }
    data.faceVertexIndices.insert(
        data.faceVertexIndices.end(), cap.faceVertexIndices.begin(), cap.faceVertexIndices.end());
}

MeshData generatePlane(
    double halfScale, int64_t uPatches = 1, int64_t vPatches = 1, int64_t uVerticesScale = 1, int64_t vVerticesScale = 1)
{
    return buildQuadGrid({ 0.0, 0.0, 0.0 }, halfScale, resolvePatchCount(uPatches, uVerticesScale, 1),
                         resolvePatchCount(vPatches, vVerticesScale, 1), GridAxis::eZ);
}

MeshData generateDisk(double halfScale,
                      int64_t uPatches = 32,
                      int64_t vPatches = 1,
                      int64_t uVerticesScale = 1,
                      int64_t vVerticesScale = 1)
{
    return buildDisk({ 0.0, 0.0, 0.0 }, resolvePatchCount(uPatches, uVerticesScale, 3),
                     resolvePatchCount(vPatches, vVerticesScale, 1), halfScale);
}

MeshData generateSphere(double halfScale,
                        int64_t uPatches = 32,
                        int64_t vPatches = 16,
                        int64_t uVerticesScale = 1,
                        int64_t vVerticesScale = 1)
{
    uPatches = resolvePatchCount(uPatches, uVerticesScale, 3);
    vPatches = resolvePatchCount(vPatches, vVerticesScale, 2);

    const double uDelta = 1.0 / static_cast<double>(uPatches);
    const double vDelta = 1.0 / static_cast<double>(vPatches);
    const int64_t uVertexCount = uPatches;
    const int64_t vVertexCount = vPatches + 1;

    // Both poles are single points; the rings in between carry 'uVertexCount' points each.
    MeshData data;
    appendVector3(data.points, scaleAndOffsetPoint({ 0.0, 0.0, -1.0 }, { 0.0, 0.0, 0.0 }, halfScale));
    for (int64_t j = 1; j < vVertexCount - 1; ++j)
    {
        const double phi = (static_cast<double>(j) * vDelta - 0.5) * g_kPi;
        const double cosinePhi = std::cos(phi);
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            const double theta = static_cast<double>(i) * uDelta * 2.0 * g_kPi;
            appendVector3(data.points, scaleAndOffsetPoint(
                                           { cosinePhi * std::cos(theta), cosinePhi * std::sin(theta), std::sin(phi) },
                                           { 0.0, 0.0, 0.0 }, halfScale));
        }
    }
    appendVector3(data.points, scaleAndOffsetPoint({ 0.0, 0.0, 1.0 }, { 0.0, 0.0, 0.0 }, halfScale));

    const int64_t northPoleIndex = static_cast<int64_t>(data.points.size() / 3) - 1;
    const auto calculateIndex = [&](int64_t i, int64_t j) -> int64_t
    {
        if (j == 0)
        {
            return 0;
        }
        if (j == vVertexCount - 1)
        {
            return northPoleIndex;
        }
        return (j - 1) * uVertexCount + (i < uVertexCount ? i : 0) + 1;
    };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2 {
        return { static_cast<double>(i) * uDelta, static_cast<double>(j) * vDelta };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const int64_t index00 = calculateIndex(i, j);
            const int64_t index10 = calculateIndex(i + 1, j);
            const int64_t index11 = calculateIndex(i + 1, j + 1);
            const int64_t index01 = calculateIndex(i, j + 1);

            // At either pole two of the corners collapse onto the same point, leaving a triangle.
            std::array<int64_t, 4> faceIndices{};
            std::array<Vector2, 4> faceCoordinates{};
            std::size_t faceVertexCount = 0;
            if (index11 == index01)
            {
                faceIndices = { index00, index10, index01 };
                faceCoordinates = { calculateTextureCoordinate(i, j), calculateTextureCoordinate(i + 1, j),
                                    calculateTextureCoordinate(i, j + 1) };
                faceVertexCount = 3;
            }
            else if (index00 == index10)
            {
                faceIndices = { index00, index11, index01 };
                faceCoordinates = { calculateTextureCoordinate(i, j), calculateTextureCoordinate(i + 1, j + 1),
                                    calculateTextureCoordinate(i, j + 1) };
                faceVertexCount = 3;
            }
            else
            {
                faceIndices = { index00, index10, index11, index01 };
                faceCoordinates = { calculateTextureCoordinate(i, j), calculateTextureCoordinate(i + 1, j),
                                    calculateTextureCoordinate(i + 1, j + 1), calculateTextureCoordinate(i, j + 1) };
                faceVertexCount = 4;
            }

            data.faceVertexIndices.insert(
                data.faceVertexIndices.end(), faceIndices.begin(), faceIndices.begin() + faceVertexCount);
            data.faceVertexCounts.push_back(static_cast<int64_t>(faceVertexCount));
            for (std::size_t vertex = 0; vertex < faceVertexCount; ++vertex)
            {
                appendVector2(data.textureCoordinates, faceCoordinates[vertex]);
            }
            // On a sphere centered on the origin the position also points along the surface normal.
            for (std::size_t vertex = 0; vertex < faceVertexCount; ++vertex)
            {
                appendVector3(data.normals, normalized(readVector3(data.points, faceIndices[vertex])));
            }
        }
    }
    return data;
}

MeshData generateTorus(double halfScale,
                       int64_t uPatches = 32,
                       int64_t vPatches = 32,
                       int64_t uVerticesScale = 1,
                       int64_t vVerticesScale = 1)
{
    uPatches = resolvePatchCount(uPatches, uVerticesScale, 3);
    vPatches = resolvePatchCount(vPatches, vVerticesScale, 3);

    constexpr double holeRadius = 1.0;
    constexpr double tubeRadius = 0.5;
    const double uDelta = 1.0 / static_cast<double>(uPatches);
    const double vDelta = 1.0 / static_cast<double>(vPatches);
    const int64_t uVertexCount = uPatches;
    const int64_t vVertexCount = vPatches;

    MeshData data;
    std::vector<Vector3> vertexNormals;
    vertexNormals.reserve(static_cast<std::size_t>(uVertexCount * vVertexCount));
    for (int64_t j = 0; j < vVertexCount; ++j)
    {
        const double phi = static_cast<double>(j) * vDelta * 2.0 * g_kPi - 0.5 * g_kPi;
        const double tubeCosinePhi = tubeRadius * std::cos(phi);
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            const double theta = static_cast<double>(i) * uDelta * 2.0 * g_kPi;
            const Vector3 point = { (holeRadius + tubeCosinePhi) * std::cos(theta),
                                    (holeRadius + tubeCosinePhi) * std::sin(theta), tubeRadius * std::sin(phi) };
            // The normal points from the center of the tube out to the surface.
            const Vector3 tubeCenter = { holeRadius * std::cos(theta), holeRadius * std::sin(theta), 0.0 };
            vertexNormals.push_back(
                normalized({ point[0] - tubeCenter[0], point[1] - tubeCenter[1], point[2] - tubeCenter[2] }));
            appendVector3(data.points, scaleAndOffsetPoint(point, { 0.0, 0.0, 0.0 }, halfScale));
        }
    }

    // Both directions wrap around, so the last patch of each row and column closes back onto the first vertex.
    const auto calculateIndex = [&](int64_t i, int64_t j)
    { return (j < vVertexCount ? j : 0) * uVertexCount + (i < uVertexCount ? i : 0); };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2
    {
        return { i < uVertexCount ? static_cast<double>(i) * uDelta : 1.0,
                 j < vVertexCount ? static_cast<double>(j) * vDelta : 1.0 };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const std::array<int64_t, 4> faceIndices = { calculateIndex(i, j), calculateIndex(i + 1, j),
                                                         calculateIndex(i + 1, j + 1), calculateIndex(i, j + 1) };
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
            data.faceVertexIndices.insert(data.faceVertexIndices.end(), faceIndices.begin(), faceIndices.end());
            data.faceVertexCounts.push_back(4);
            for (int64_t index : faceIndices)
            {
                appendVector3(data.normals, vertexNormals[static_cast<std::size_t>(index)]);
            }
        }
    }
    return data;
}

// The cone is the only primitive whose reference tessellation setting does not default to 1: 'v_scale' defaults to 3,
// so an untouched cone is three patches tall.
MeshData generateCone(double halfScale,
                      int64_t uPatches = 64,
                      int64_t vPatches = 1,
                      int64_t wPatches = 1,
                      int64_t uVerticesScale = 1,
                      int64_t vVerticesScale = 3,
                      int64_t wVerticesScale = 1)
{
    uPatches = resolvePatchCount(uPatches, uVerticesScale, 3);
    vPatches = resolvePatchCount(vPatches, vVerticesScale, 1);
    wPatches = resolvePatchCount(wPatches, wVerticesScale, 1);

    constexpr double height = 2.0;
    // The tip is pulled just short of v = 1 to keep the surface derivatives that define the normals finite.
    constexpr double apexEpsilon = 0.00001;
    const double uDelta = 1.0 / static_cast<double>(uPatches);
    const double vDelta = (1.0 - apexEpsilon) / static_cast<double>(vPatches);
    const int64_t uVertexCount = uPatches;
    const int64_t vVertexCount = vPatches + 1;

    MeshData data;
    std::vector<Vector3> vertexNormals;
    vertexNormals.reserve(static_cast<std::size_t>(uVertexCount * vVertexCount));
    for (int64_t j = 0; j < vVertexCount; ++j)
    {
        const double v = static_cast<double>(j) * vDelta;
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            const double theta = static_cast<double>(i) * uDelta * 2.0 * g_kPi;
            const double x = (1.0 - v) * std::cos(theta);
            const double y = (1.0 - v) * std::sin(theta);
            const Vector3 partialU = { -2.0 * g_kPi * y, 2.0 * g_kPi * x, 0.0 };
            const Vector3 partialV = { -x / (1.0 - v), -y / (1.0 - v), height };
            vertexNormals.push_back(normalizedCrossProduct(partialU, partialV));
            appendVector3(data.points, scaleAndOffsetPoint({ x, y, v * height - 1.0 }, { 0.0, 0.0, 0.0 }, halfScale));
        }
    }

    const auto calculateIndex = [&](int64_t i, int64_t j) { return j * uVertexCount + (i < uVertexCount ? i : 0); };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2
    {
        return { i < uVertexCount ? static_cast<double>(i) * uDelta : 1.0,
                 j != vVertexCount - 1 ? static_cast<double>(j) * vDelta : 1.0 };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const std::array<int64_t, 4> faceIndices = { calculateIndex(i, j), calculateIndex(i + 1, j),
                                                         calculateIndex(i + 1, j + 1), calculateIndex(i, j + 1) };
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
            data.faceVertexIndices.insert(data.faceVertexIndices.end(), faceIndices.begin(), faceIndices.end());
            data.faceVertexCounts.push_back(4);
            for (int64_t index : faceIndices)
            {
                appendVector3(data.normals, vertexNormals[static_cast<std::size_t>(index)]);
            }
        }
    }

    // The apex ring has collapsed to a near-point, so its cap needs a single ring; the base gets the full one.
    const int64_t apexRimStartIndex = static_cast<int64_t>(data.points.size() / 3) - uVertexCount;
    appendCap(data, { 0.0, 0.0, 1.0 - apexEpsilon }, apexRimStartIndex, uPatches, 1, halfScale, false);
    appendCap(data, { 0.0, 0.0, -1.0 }, 0, uPatches, wPatches, halfScale, true);
    return data;
}

MeshData generateCylinder(double halfScale,
                          int64_t uPatches = 32,
                          int64_t vPatches = 1,
                          int64_t wPatches = 1,
                          int64_t uVerticesScale = 1,
                          int64_t vVerticesScale = 1,
                          int64_t wVerticesScale = 1)
{
    uPatches = resolvePatchCount(uPatches, uVerticesScale, 3);
    vPatches = resolvePatchCount(vPatches, vVerticesScale, 1);
    wPatches = resolvePatchCount(wPatches, wVerticesScale, 1);

    const double uDelta = 1.0 / static_cast<double>(uPatches);
    const double vDelta = 1.0 / static_cast<double>(vPatches);
    const int64_t uVertexCount = uPatches;
    const int64_t vVertexCount = vPatches + 1;

    MeshData data;
    const std::vector<Vector3> circlePoints = generateCirclePoints(uVertexCount, uDelta);
    for (int64_t j = 0; j < vVertexCount; ++j)
    {
        const double v = static_cast<double>(j) * vDelta;
        for (int64_t i = 0; i < uVertexCount; ++i)
        {
            Vector3 point = circlePoints[static_cast<std::size_t>(i)];
            point[2] = 2.0 * (v - 0.5);
            appendVector3(data.points, scaleAndOffsetPoint(point, { 0.0, 0.0, 0.0 }, halfScale));
        }
    }

    const auto calculateIndex = [&](int64_t i, int64_t j)
    { return (j < vVertexCount ? j : 0) * uVertexCount + (i < uVertexCount ? i : 0); };
    const auto calculateTextureCoordinate = [&](int64_t i, int64_t j) -> Vector2
    {
        return { i < uVertexCount ? static_cast<double>(i) * uDelta : 1.0,
                 j < vVertexCount ? static_cast<double>(j) * vDelta : 1.0 };
    };

    for (int64_t j = 0; j < vPatches; ++j)
    {
        for (int64_t i = 0; i < uPatches; ++i)
        {
            const std::array<int64_t, 4> faceIndices = { calculateIndex(i, j), calculateIndex(i + 1, j),
                                                         calculateIndex(i + 1, j + 1), calculateIndex(i, j + 1) };
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i + 1, j + 1));
            appendVector2(data.textureCoordinates, calculateTextureCoordinate(i, j + 1));
            data.faceVertexIndices.insert(data.faceVertexIndices.end(), faceIndices.begin(), faceIndices.end());
            data.faceVertexCounts.push_back(4);
            // The side normal is the point pushed onto the XY plane, since the cylinder axis is Z.
            for (int64_t index : faceIndices)
            {
                const Vector3 point = readVector3(data.points, index);
                appendVector3(data.normals, normalized({ point[0], point[1], 0.0 }));
            }
        }
    }

    // Resolve the top rim before either cap appends points, so that it still refers to the body's last ring.
    const int64_t topRimStartIndex = static_cast<int64_t>(data.points.size() / 3) - uVertexCount;
    appendCap(data, { 0.0, 0.0, -1.0 }, 0, uPatches, wPatches, halfScale, true);
    appendCap(data, { 0.0, 0.0, 1.0 }, topRimStartIndex, uPatches, wPatches, halfScale, false);
    return data;
}

MeshData generateCube(double halfScale,
                      int64_t uPatches = 1,
                      int64_t vPatches = 1,
                      int64_t wPatches = 1,
                      int64_t uVerticesScale = 1,
                      int64_t vVerticesScale = 1,
                      int64_t wVerticesScale = 1)
{
    uPatches = resolvePatchCount(uPatches, uVerticesScale, 1);
    vPatches = resolvePatchCount(vPatches, vVerticesScale, 1);
    wPatches = resolvePatchCount(wPatches, wVerticesScale, 1);

    // Each grid is built once at the near face and reused for the opposite one by offsetting it across the cube.
    const MeshData frontGrid = buildQuadGrid({ 0.0, 0.0, halfScale }, halfScale, uPatches, vPatches, GridAxis::eZ);
    const MeshData bottomGrid = buildQuadGrid({ 0.0, -halfScale, 0.0 }, halfScale, uPatches, wPatches, GridAxis::eY);
    const MeshData leftGrid = buildQuadGrid({ -halfScale, 0.0, 0.0 }, halfScale, vPatches, wPatches, GridAxis::eX);

    MeshData data;
    // 'remapCoordinate' rotates or mirrors the grid's texture coordinates onto the face's own UV convention.
    const auto appendFace = [&](const MeshData& grid, const Vector3& offset, const Vector3& normal,
                                bool invertWindingOrder, auto&& remapCoordinate)
    {
        const int64_t baseIndex = static_cast<int64_t>(data.points.size() / 3);
        for (std::size_t i = 0; i < grid.points.size(); i += 3)
        {
            appendVector3(data.points, { grid.points[i] + offset[0], grid.points[i + 1] + offset[1],
                                         grid.points[i + 2] + offset[2] });
        }
        for (std::size_t i = 0; i < grid.normals.size(); i += 3)
        {
            appendVector3(data.normals, normal);
        }

        std::vector<int64_t> faceIndices = grid.faceVertexIndices;
        std::vector<float> coordinates = grid.textureCoordinates;
        for (int64_t& index : faceIndices)
        {
            index += baseIndex;
        }
        if (invertWindingOrder)
        {
            modifyWindingOrder(grid.faceVertexCounts, faceIndices, 1);
            modifyWindingOrder(grid.faceVertexCounts, coordinates, 2);
        }
        for (std::size_t i = 0; i < coordinates.size(); i += 2)
        {
            appendVector2(data.textureCoordinates, remapCoordinate(Vector2{ coordinates[i], coordinates[i + 1] }));
        }
        data.faceVertexIndices.insert(data.faceVertexIndices.end(), faceIndices.begin(), faceIndices.end());
        data.faceVertexCounts.insert(
            data.faceVertexCounts.end(), grid.faceVertexCounts.begin(), grid.faceVertexCounts.end());
    };

    const auto keepCoordinate = [](Vector2 coordinate) { return coordinate; };
    const auto mirrorU = [](Vector2 coordinate) { return Vector2{ 1.0 - coordinate[0], coordinate[1] }; };
    const auto mirrorV = [](Vector2 coordinate) { return Vector2{ coordinate[0], 1.0 - coordinate[1] }; };
    const auto swapUV = [](Vector2 coordinate) { return Vector2{ coordinate[1], coordinate[0] }; };
    const auto swapAndMirrorUV = [](Vector2 coordinate) { return Vector2{ 1.0 - coordinate[1], coordinate[0] }; };

    appendFace(frontGrid, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 }, false, keepCoordinate);
    appendFace(frontGrid, { 0.0, 0.0, -2.0 * halfScale }, { 0.0, 0.0, -1.0 }, true, mirrorU);
    appendFace(bottomGrid, { 0.0, 2.0 * halfScale, 0.0 }, { 0.0, 1.0, 0.0 }, false, keepCoordinate);
    appendFace(bottomGrid, { 0.0, 0.0, 0.0 }, { 0.0, -1.0, 0.0 }, true, mirrorV);
    appendFace(leftGrid, { 0.0, 0.0, 0.0 }, { -1.0, 0.0, 0.0 }, false, swapUV);
    appendFace(leftGrid, { 2.0 * halfScale, 0.0, 0.0 }, { 1.0, 0.0, 0.0 }, true, swapAndMirrorUV);

    // Weld the vertices that adjacent faces duplicate along the cube's edges. Cube point counts stay small enough
    // that the quadratic scan costs less than building a spatial index.
    constexpr double weldToleranceSquared = 1e-6 * 1e-6;
    const int64_t pointCount = static_cast<int64_t>(data.points.size() / 3);
    std::vector<int64_t> remappedIndices(static_cast<std::size_t>(pointCount), -1);
    std::vector<float> weldedPoints;
    weldedPoints.reserve(data.points.size());
    for (int64_t i = 0; i < pointCount; ++i)
    {
        if (remappedIndices[static_cast<std::size_t>(i)] != -1)
        {
            continue;
        }
        const Vector3 point = readVector3(data.points, i);
        appendVector3(weldedPoints, point);
        const int64_t weldedIndex = static_cast<int64_t>(weldedPoints.size() / 3) - 1;
        remappedIndices[static_cast<std::size_t>(i)] = weldedIndex;
        for (int64_t j = i + 1; j < pointCount; ++j)
        {
            if (remappedIndices[static_cast<std::size_t>(j)] != -1)
            {
                continue;
            }
            const Vector3 other = readVector3(data.points, j);
            const Vector3 difference = { other[0] - point[0], other[1] - point[1], other[2] - point[2] };
            const double distanceSquared =
                difference[0] * difference[0] + difference[1] * difference[1] + difference[2] * difference[2];
            if (distanceSquared <= weldToleranceSquared)
            {
                remappedIndices[static_cast<std::size_t>(j)] = weldedIndex;
            }
        }
    }
    for (int64_t& index : data.faceVertexIndices)
    {
        index = remappedIndices[static_cast<std::size_t>(index)];
    }
    data.points = std::move(weldedPoints);
    return data;
}

// Axis-aligned bounds of 'points' as the flat (min, max) pair that the USD 'extent' attribute expects.
std::vector<float> computeExtent(const std::vector<float>& points)
{
    std::vector<float> extent(6, 0.0f);
    if (points.empty())
    {
        return extent;
    }
    for (std::size_t component = 0; component < 3; ++component)
    {
        extent[component] = points[component];
        extent[component + 3] = points[component];
    }
    for (std::size_t i = 3; i < points.size(); i += 3)
    {
        for (std::size_t component = 0; component < 3; ++component)
        {
            extent[component] = std::min(extent[component], points[i + component]);
            extent[component + 3] = std::max(extent[component + 3], points[i + component]);
        }
    }
    return extent;
}

MeshData generatePrimitive(const std::string& primitive, double halfScale)
{
    if (primitive == "Cone")
    {
        return generateCone(halfScale);
    }
    if (primitive == "Cube")
    {
        return generateCube(halfScale);
    }
    if (primitive == "Cylinder")
    {
        return generateCylinder(halfScale);
    }
    if (primitive == "Disk")
    {
        return generateDisk(halfScale);
    }
    if (primitive == "Plane")
    {
        return generatePlane(halfScale);
    }
    if (primitive == "Sphere")
    {
        return generateSphere(halfScale);
    }
    if (primitive == "Torus")
    {
        return generateTorus(halfScale);
    }
    throw std::invalid_argument("Invalid primitive: '" + primitive + "'");
}

} // namespace

Mesh::Mesh(const std::variant<std::string, std::vector<std::string>>& paths,
           const std::optional<std::variant<std::string, std::vector<std::string>>>& primitives,
           const std::optional<ColorType>& colors,
           bool resetXformOpProperties)
    : Xform()
{
    // Get or create mesh prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get mesh prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<bool> isMesh = this->isA("Mesh").flatten().get<std::vector<bool>>();
        for (std::size_t i = 0; i < m_paths.size(); ++i)
        {
            if (!isMesh[i])
            {
                throw std::runtime_error("The wrapped prim at path '" + m_paths[i] + "' is not a USD Mesh");
            }
        }
    }
    // Create mesh prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        if (primitives.has_value())
        {
            const std::vector<std::string> primitiveNames = _resolveStringList(*primitives, std::nullopt);
            // The reference primitives are authored with a 50 cm half extent; express it in stage units.
            const double metersPerUnit = static_cast<double>(std::get<0>(this->getStage().getUnits()));
            const double halfScale = 0.5 / (metersPerUnit > 0.0 ? metersPerUnit : 0.01);
            // Generate every distinct primitive before defining any prim, so that an invalid name leaves the stage
            // untouched and a broadcast name is only tessellated once.
            std::unordered_map<std::string, MeshData> geometries;
            for (const std::string& primitive : primitiveNames)
            {
                if (geometries.find(primitive) == geometries.end())
                {
                    geometries.emplace(primitive, generatePrimitive(primitive, halfScale));
                }
            }
            for (std::size_t i = 0; i < m_paths.size(); ++i)
            {
                this->getStage().definePrim(m_paths[i], "Mesh");
                const MeshData& data = geometries.at(primitiveNames[i]);
                const Prim prim(m_paths[i], /*resolvePaths=*/false);
                prim.setAttributeValues(
                    "points",
                    array::Array(data.points).reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 }, int64_t{ 3 } })));
                prim.setAttributeValues(
                    "normals",
                    array::Array(data.normals).reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 }, int64_t{ 3 } })));
                prim.setAttributeValues(
                    "faceVertexIndices",
                    array::Array(data.faceVertexIndices).reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 } })));
                prim.setAttributeValues(
                    "faceVertexCounts",
                    array::Array(data.faceVertexCounts).reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 } })));
                // TODO: Set the 'interpolation' metadata of 'normals' and 'primvars:st' to 'faceVarying'. USD stores it
                //       as attribute metadata, which no stage backend exposes yet, so both currently fall back to
                //       'constant'.
                prim.createAttribute("primvars:st", "texCoord2f[]");
                prim.setAttributeValues(
                    "primvars:st", array::Array(data.textureCoordinates)
                                       .reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 }, int64_t{ 2 } })));
                prim.setAttributeValues("subdivisionScheme", std::vector<std::string>{ "none" });
                prim.setAttributeValues(
                    "extent", array::Array(computeExtent(data.points))
                                  .reshape(array::Shape({ int64_t{ 1 }, int64_t{ 2 }, int64_t{ 3 } })));
            }
        }
        else
        {
            for (const std::string& path : m_paths)
            {
                this->getStage().definePrim(path, "Mesh");
            }
        }
    }
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    if (colors.has_value())
    {
        this->setDisplayColors(*colors);
    }
}

std::vector<size_t> Mesh::numFaces()
{
    std::vector<size_t> result;
    result.reserve(m_paths.size());
    for (const std::string& path : m_paths)
    {
        const Prim prim(path, /*resolvePaths=*/false);
        auto faceVertexCounts = std::get<array::Array>(prim.getAttributeValues("faceVertexCounts"));
        result.push_back(faceVertexCounts.at(0).size());
    }
    return result;
}

void Mesh::setPoints(const std::vector<array::Array>& points, const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    for (std::size_t i = 0; i < indexValues.size(); ++i)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(indexValues[i])], /*resolvePaths=*/false);
        prim.setAttributeValues("points", resolveItem(points, i, indexValues.size(), "points")
                                              .reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 }, int64_t{ 3 } })));
    }
}

std::vector<array::Array> Mesh::getPoints(const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<array::Array> result;
    result.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(index)], /*resolvePaths=*/false);
        result.push_back(std::get<array::Array>(prim.getAttributeValues("points")).at(0));
    }
    return result;
}

void Mesh::setNormals(const std::vector<array::Array>& normals, const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    for (std::size_t i = 0; i < indexValues.size(); ++i)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(indexValues[i])], /*resolvePaths=*/false);
        prim.setAttributeValues("normals", resolveItem(normals, i, indexValues.size(), "normals")
                                               .reshape(array::Shape({ int64_t{ 1 }, int64_t{ -1 }, int64_t{ 3 } })));
    }
}

std::vector<array::Array> Mesh::getNormals(const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<array::Array> result;
    result.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(index)], /*resolvePaths=*/false);
        result.push_back(std::get<array::Array>(prim.getAttributeValues("normals")).at(0));
    }
    return result;
}

void Mesh::setFaceSpecifications(
    const std::optional<std::vector<array::Array>>& vertexIndices,
    const std::optional<std::vector<array::Array>>& vertexCounts,
    const std::optional<std::variant<std::string, std::vector<std::string>>>& varyingLinearInterpolations,
    const std::optional<std::vector<array::Array>>& holeIndices,
    const std::optional<array::Array>& indices)
{
    if (!vertexIndices.has_value() && !vertexCounts.has_value() && !varyingLinearInterpolations.has_value() &&
        !holeIndices.has_value())
    {
        throw std::invalid_argument(
            "All 'vertexIndices', 'vertexCounts', 'varyingLinearInterpolations' and 'holeIndices' are not defined. "
            "Define at least one of them");
    }

    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    for (std::size_t i = 0; i < indexValues.size(); ++i)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(indexValues[i])], /*resolvePaths=*/false);
        const array::Shape shape({ int64_t{ 1 }, int64_t{ -1 } });
        if (vertexIndices.has_value())
        {
            prim.setAttributeValues("faceVertexIndices",
                                    resolveItem(*vertexIndices, i, indexValues.size(), "vertexIndices").reshape(shape));
        }
        if (vertexCounts.has_value())
        {
            prim.setAttributeValues(
                "faceVertexCounts", resolveItem(*vertexCounts, i, indexValues.size(), "vertexCounts").reshape(shape));
        }
        if (holeIndices.has_value())
        {
            prim.setAttributeValues(
                "holeIndices", resolveItem(*holeIndices, i, indexValues.size(), "holeIndices").reshape(shape));
        }
    }
    if (varyingLinearInterpolations.has_value())
    {
        this->setAttributeValues(
            "faceVaryingLinearInterpolation", _resolveStringList(*varyingLinearInterpolations, indices), indices);
    }
}

std::tuple<std::vector<array::Array>, std::vector<array::Array>, std::vector<std::string>, std::vector<array::Array>> Mesh::
    getFaceSpecifications(const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<array::Array> vertexIndices, vertexCounts, holeIndices;
    vertexIndices.reserve(indexValues.size());
    vertexCounts.reserve(indexValues.size());
    holeIndices.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(index)], /*resolvePaths=*/false);
        vertexIndices.push_back(std::get<array::Array>(prim.getAttributeValues("faceVertexIndices")).at(0));
        vertexCounts.push_back(std::get<array::Array>(prim.getAttributeValues("faceVertexCounts")).at(0));
        holeIndices.push_back(std::get<array::Array>(prim.getAttributeValues("holeIndices")).at(0));
    }
    return { std::move(vertexIndices), std::move(vertexCounts),
             std::get<std::vector<std::string>>(this->getAttributeValues("faceVaryingLinearInterpolation", indices)),
             std::move(holeIndices) };
}

void Mesh::setCreaseSpecifications(const std::vector<array::Array>& creaseIndices,
                                   const std::vector<array::Array>& creaseLengths,
                                   const std::vector<array::Array>& creaseSharpnesses,
                                   const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    for (std::size_t i = 0; i < indexValues.size(); ++i)
    {
        const array::Array& creaseIndex = resolveItem(creaseIndices, i, indexValues.size(), "creaseIndices");
        const array::Array& creaseLength = resolveItem(creaseLengths, i, indexValues.size(), "creaseLengths");
        const array::Array& creaseSharpness = resolveItem(creaseSharpnesses, i, indexValues.size(), "creaseSharpnesses");

        // Each crease is at least one edge long, so its length accounts for one more point than edges.
        const std::vector<int64_t> lengths = creaseLength.flatten().get<std::vector<int64_t>>();
        const int64_t pointCount = std::accumulate(lengths.begin(), lengths.end(), int64_t{ 0 });
        const int64_t edgeCount = pointCount - static_cast<int64_t>(lengths.size());
        if (pointCount != static_cast<int64_t>(creaseIndex.size()))
        {
            throw std::invalid_argument("The sum of the elements of 'creaseLengths' (" + std::to_string(pointCount) +
                                        ") (at index " + std::to_string(i) +
                                        ") is not equal to the number of elements of 'creaseIndices' (" +
                                        std::to_string(creaseIndex.size()) + ")");
        }
        if (creaseSharpness.size() != lengths.size() && static_cast<int64_t>(creaseSharpness.size()) != edgeCount)
        {
            throw std::invalid_argument(
                "The number of elements of 'creaseSharpnesses' (" + std::to_string(creaseSharpness.size()) +
                ") (at index " + std::to_string(i) + ") is not equal to the number of elements of 'creaseLengths' (" +
                std::to_string(lengths.size()) + ") or the sum over all X of (creaseLengths[X] - 1) (" +
                std::to_string(edgeCount) + ")");
        }

        const Prim prim(m_paths[static_cast<std::size_t>(indexValues[i])], /*resolvePaths=*/false);
        const array::Shape shape({ int64_t{ 1 }, int64_t{ -1 } });
        prim.setAttributeValues("creaseIndices", creaseIndex.reshape(shape));
        prim.setAttributeValues("creaseLengths", creaseLength.reshape(shape));
        prim.setAttributeValues("creaseSharpnesses", creaseSharpness.reshape(shape));
    }
}

std::tuple<std::vector<array::Array>, std::vector<array::Array>, std::vector<array::Array>> Mesh::getCreaseSpecifications(
    const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<array::Array> creaseIndices, creaseLengths, creaseSharpnesses;
    creaseIndices.reserve(indexValues.size());
    creaseLengths.reserve(indexValues.size());
    creaseSharpnesses.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(index)], /*resolvePaths=*/false);
        creaseIndices.push_back(std::get<array::Array>(prim.getAttributeValues("creaseIndices")).at(0));
        creaseLengths.push_back(std::get<array::Array>(prim.getAttributeValues("creaseLengths")).at(0));
        creaseSharpnesses.push_back(std::get<array::Array>(prim.getAttributeValues("creaseSharpnesses")).at(0));
    }
    return { std::move(creaseIndices), std::move(creaseLengths), std::move(creaseSharpnesses) };
}

void Mesh::setCornerSpecifications(const std::vector<array::Array>& cornerIndices,
                                   const std::vector<array::Array>& cornerSharpnesses,
                                   const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    for (std::size_t i = 0; i < indexValues.size(); ++i)
    {
        const array::Array& cornerIndex = resolveItem(cornerIndices, i, indexValues.size(), "cornerIndices");
        const array::Array& cornerSharpness = resolveItem(cornerSharpnesses, i, indexValues.size(), "cornerSharpnesses");
        if (cornerIndex.size() != cornerSharpness.size())
        {
            throw std::invalid_argument("Items from 'cornerIndices' and 'cornerSharpnesses' (at index " +
                                        std::to_string(i) + ") have different sizes (" +
                                        std::to_string(cornerIndex.size()) +
                                        " != " + std::to_string(cornerSharpness.size()) + ")");
        }
        const Prim prim(m_paths[static_cast<std::size_t>(indexValues[i])], /*resolvePaths=*/false);
        const array::Shape shape({ int64_t{ 1 }, int64_t{ -1 } });
        prim.setAttributeValues("cornerIndices", cornerIndex.reshape(shape));
        prim.setAttributeValues("cornerSharpnesses", cornerSharpness.reshape(shape));
    }
}

std::tuple<std::vector<array::Array>, std::vector<array::Array>> Mesh::getCornerSpecifications(
    const std::optional<array::Array>& indices)
{
    const std::vector<int64_t> indexValues = _resolveIndexValues(indices);
    std::vector<array::Array> cornerIndices, cornerSharpnesses;
    cornerIndices.reserve(indexValues.size());
    cornerSharpnesses.reserve(indexValues.size());
    for (int64_t index : indexValues)
    {
        const Prim prim(m_paths[static_cast<std::size_t>(index)], /*resolvePaths=*/false);
        cornerIndices.push_back(std::get<array::Array>(prim.getAttributeValues("cornerIndices")).at(0));
        cornerSharpnesses.push_back(std::get<array::Array>(prim.getAttributeValues("cornerSharpnesses")).at(0));
    }
    return { std::move(cornerIndices), std::move(cornerSharpnesses) };
}

void Mesh::setSubdivisionSpecifications(
    const std::optional<std::variant<std::string, std::vector<std::string>>>& subdivisionSchemes,
    const std::optional<std::variant<std::string, std::vector<std::string>>>& interpolateBoundaries,
    const std::optional<std::variant<std::string, std::vector<std::string>>>& triangleSubdivisionRules,
    const std::optional<array::Array>& indices)
{
    if (!subdivisionSchemes.has_value() && !interpolateBoundaries.has_value() && !triangleSubdivisionRules.has_value())
    {
        throw std::invalid_argument(
            "All 'subdivisionSchemes', 'interpolateBoundaries' and 'triangleSubdivisionRules' are not defined. "
            "Define at least one of them");
    }
    if (subdivisionSchemes.has_value())
    {
        this->setAttributeValues("subdivisionScheme", _resolveStringList(*subdivisionSchemes, indices), indices);
    }
    if (interpolateBoundaries.has_value())
    {
        this->setAttributeValues("interpolateBoundary", _resolveStringList(*interpolateBoundaries, indices), indices);
    }
    if (triangleSubdivisionRules.has_value())
    {
        this->setAttributeValues(
            "triangleSubdivisionRule", _resolveStringList(*triangleSubdivisionRules, indices), indices);
    }
}

std::tuple<std::vector<std::string>, std::vector<std::string>, std::vector<std::string>> Mesh::getSubdivisionSpecifications(
    const std::optional<array::Array>& indices)
{
    return { std::get<std::vector<std::string>>(this->getAttributeValues("subdivisionScheme", indices)),
             std::get<std::vector<std::string>>(this->getAttributeValues("interpolateBoundary", indices)),
             std::get<std::vector<std::string>>(this->getAttributeValues("triangleSubdivisionRule", indices)) };
}

void Mesh::setDisplayColors(const ColorType& colors, const std::optional<array::Array>& indices)
{
    // TODO: implementation.
    (void)colors;
    (void)indices;
}

array::Array Mesh::getDisplayColors(const std::optional<array::Array>& indices)
{
    // TODO: implementation.
    (void)indices;
    return array::Array(0);
}

void Mesh::updateExtents()
{
    // TODO: Implement and call it when setting values.
}

array::Array Mesh::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("Mesh");
}

} // namespace objects
} // namespace foundation
} // namespace isaacsim
