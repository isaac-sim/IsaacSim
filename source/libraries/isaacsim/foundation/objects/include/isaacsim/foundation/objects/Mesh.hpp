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

#include <isaacsim/foundation/objects/Xform.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

/// @brief Variant type for display colors: a single color token, a list of color tokens, or a numeric RGB array.
using ColorType = std::variant<std::string, std::vector<std::string>, array::Array>;

/**
 * @class Mesh
 * @brief Wrapper over one or more USD Mesh (points connected into edges and faces) prims.
 * @details
 * Wraps existing USD Mesh prims or creates new ones, depending on whether the given paths already
 * resolve on the active stage:
 *
 * - If the paths exist, a wrapper is placed over the USD Mesh prims found there.
 * - If the paths do not exist, USD Mesh prims are created at each path and then wrapped.
 *
 * Mesh attributes are ragged: each prim holds its own number of points, faces, creases and corners.
 * Accessors therefore take and return one array per selected prim rather than a single batched array.
 * All methods that accept an @p indices parameter operate only on the prims at those positions; when
 * @p indices is omitted, all wrapped prims are processed. Where a method takes a list of arrays, a
 * single-element list is broadcast to every selected prim.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API Mesh : public Xform
{
public:
    /**
     * @brief Construct a Mesh wrapper and optionally configure the initial geometry.
     * @details
     * When the prims are created rather than wrapped, @p primitives selects the geometry generated at
     * each path. Supported names are @c "Cone", @c "Cube", @c "Cylinder", @c "Disk", @c "Plane",
     * @c "Sphere" and @c "Torus"; a single name is broadcast to every path. The generated primitives
     * are Z-up and are sized to a half extent of 50 cm expressed in stage units. If @p primitives is
     * omitted, empty meshes are created instead.
     *
     * @param[in] paths                  Single path or list of paths to existing or non-existing (one of
     *                                   both) USD prims. May include regular expressions.
     * @param[in] primitives             Names of the primitives to generate (shape @c (N,)). Used only when
     *                                   creating prims; ignored when wrapping existing ones. Optional.
     * @param[in] colors                 Initial display colors. Optional.
     * @param[in] resetXformOpProperties Whether to normalize the xformOp stack to translate/orient/scale.
     * @throws std::runtime_error if a wrapped prim is not a USD Mesh.
     * @throws std::invalid_argument if a primitive name is not supported, or if the number of primitives
     *         is neither one nor the number of paths.
     */
    Mesh(const std::variant<std::string, std::vector<std::string>>& paths,
         // Mesh
         const std::optional<std::variant<std::string, std::vector<std::string>>>& primitives = std::nullopt,
         const std::optional<ColorType>& colors = std::nullopt,
         // XformPrim
         bool resetXformOpProperties = true);
    ~Mesh() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Get the number of faces of all wrapped meshes.
     * @return Face counts, as given by the size of each prim's @c faceVertexCounts array, one per wrapped prim.
     */
    std::vector<size_t> numFaces();

    /**
     * @brief Set the mesh points (in local space) of the selected prims.
     * @param[in] points  List (shape @c (N,)) of point arrays (shape @c (number of points, 3)).
     *                    Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if @p points holds neither one array nor one per selected prim.
     */
    void setPoints(const std::vector<array::Array>& points, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the mesh points (in local space) of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return List (shape @c (N,)) of point arrays (shape @c (number of points, 3)).
     */
    std::vector<array::Array> getPoints(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the mesh normals (object-space orientation for individual points) of the selected prims.
     * @note Normals should not be authored on a subdivided USD Mesh, since the subdivision algorithm
     *       defines its own normals.
     * @param[in] normals List (shape @c (N,)) of normal arrays (shape @c (number of normals, 3)).
     *                    Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if @p normals holds neither one array nor one per selected prim.
     */
    void setNormals(const std::vector<array::Array>& normals, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the mesh normals (object-space orientation for individual points) of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return List (shape @c (N,)) of normal arrays (shape @c (number of normals, 3)).
     */
    std::vector<array::Array> getNormals(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the face (3D model flat surface) specifications of the selected prims.
     * @param[in] vertexIndices               List (shape @c (N,)) of point indices for each vertex of each face
     *                                        (shape @c (sum of all elements of the vertexCounts,)). Optional.
     * @param[in] vertexCounts                List (shape @c (N,)) of the number of vertices in each face, which is
     *                                        also the number of consecutive entries in @c faceVertexIndices that
     *                                        define the face (shape @c (number of faces,)). Optional.
     * @param[in] varyingLinearInterpolations Face-varying interpolation rules in the interior of face-varying
     *                                        regions and at the boundaries for subdivision surfaces
     *                                        (shape @c (N,)). Optional.
     * @param[in] holeIndices                 List (shape @c (N,)) of indices of all faces to treat as holes, that
     *                                        is, made invisible (shape @c (up to the number of faces,)). Optional.
     * @param[in] indices                     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if none of @p vertexIndices, @p vertexCounts,
     *         @p varyingLinearInterpolations and @p holeIndices is defined.
     */
    void setFaceSpecifications(
        const std::optional<std::vector<array::Array>>& vertexIndices = std::nullopt,
        const std::optional<std::vector<array::Array>>& vertexCounts = std::nullopt,
        const std::optional<std::variant<std::string, std::vector<std::string>>>& varyingLinearInterpolations = std::nullopt,
        const std::optional<std::vector<array::Array>>& holeIndices = std::nullopt,
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the face (3D model flat surface) specifications of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Four-element tuple. 1) List of point indices for each vertex of each face.
     *         2) List of the number of vertices in each face. 3) List of face-varying interpolation rules.
     *         4) List of indices of all face holes.
     */
    std::tuple<std::vector<array::Array>, std::vector<array::Array>, std::vector<std::string>, std::vector<array::Array>> getFaceSpecifications(
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the crease (set of adjacent sharpened edges) specifications of the selected prims.
     * @param[in] creaseIndices     List (shape @c (N,)) of point indices grouped into sets of successive pairs
     *                              that identify the edges to crease
     *                              (shape @c (sum of all elements of the creaseLengths,)). Broadcast rules apply.
     * @param[in] creaseLengths     List (shape @c (N,)) of the number of points of each crease, whose indices are
     *                              successively laid out in @c creaseIndices (shape @c (number of creases,)).
     *                              Since each crease is at least one edge long, every element is at least two.
     *                              Broadcast rules apply.
     * @param[in] creaseSharpnesses List (shape @c (N,)) of per-crease or per-edge sharpness values
     *                              (shape @c (number of creases,) or
     *                              @c (sum over all X of (creaseLengths[X] - 1),)). Broadcast rules apply.
     * @param[in] indices           Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if the sum of the elements of @p creaseLengths is not equal to the number of
     *         elements of @p creaseIndices, or if the number of elements of @p creaseSharpnesses matches neither
     *         the number of elements of @p creaseLengths nor the sum over all X of @c (creaseLengths[X] - 1).
     */
    void setCreaseSpecifications(const std::vector<array::Array>& creaseIndices,
                                 const std::vector<array::Array>& creaseLengths,
                                 const std::vector<array::Array>& creaseSharpnesses,
                                 const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the crease (set of adjacent sharpened edges) specifications of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Three-element tuple. 1) List of point indices. 2) List of the number of points of each crease.
     *         3) List of sharpness values.
     */
    std::tuple<std::vector<array::Array>, std::vector<array::Array>, std::vector<array::Array>> getCreaseSpecifications(
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the corner specifications of the selected prims.
     * @param[in] cornerIndices     List (shape @c (N,)) of point indices (shape @c (number of target points,))
     *                              for which a corresponding sharpness value is given in @p cornerSharpnesses.
     *                              Broadcast rules apply.
     * @param[in] cornerSharpnesses List (shape @c (N,)) of sharpness values (shape @c (number of target points,))
     *                              associated with the points given in @p cornerIndices. Broadcast rules apply.
     * @param[in] indices           Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a pair of items from @p cornerIndices and @p cornerSharpnesses has
     *         different sizes.
     */
    void setCornerSpecifications(const std::vector<array::Array>& cornerIndices,
                                 const std::vector<array::Array>& cornerSharpnesses,
                                 const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the corner specifications of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Two-element tuple. 1) List of point indices. 2) List of sharpness values.
     */
    std::tuple<std::vector<array::Array>, std::vector<array::Array>> getCornerSpecifications(
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the subdivision specifications of the selected prims.
     * @param[in] subdivisionSchemes        Subdivision schemes (shape @c (N,)). Optional.
     * @param[in] interpolateBoundaries     Boundary interpolation rules for faces adjacent to boundary edges
     *                                      and points (shape @c (N,)). Optional.
     * @param[in] triangleSubdivisionRules  Subdivision rules for the *Catmull-Clark* scheme, used to reduce
     *                                      artifacts when subdividing triangles (shape @c (N,)). Optional.
     * @param[in] indices                   Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if none of @p subdivisionSchemes, @p interpolateBoundaries and
     *         @p triangleSubdivisionRules is defined.
     */
    void setSubdivisionSpecifications(
        const std::optional<std::variant<std::string, std::vector<std::string>>>& subdivisionSchemes = std::nullopt,
        const std::optional<std::variant<std::string, std::vector<std::string>>>& interpolateBoundaries = std::nullopt,
        const std::optional<std::variant<std::string, std::vector<std::string>>>& triangleSubdivisionRules = std::nullopt,
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the subdivision specifications of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Three-element tuple. 1) Subdivision schemes. 2) Boundary interpolation rules.
     *         3) Triangle subdivision rules.
     */
    std::tuple<std::vector<std::string>, std::vector<std::string>, std::vector<std::string>> getSubdivisionSpecifications(
        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the display colors of the selected prims.
     * @param[in] colors  Normalized RGB display colors (shape @c (N,3)) or case-insensitive string
     *                    representations. Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setDisplayColors(const ColorType& colors, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the display colors of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Normalized RGB display colors (shape @c (N,3)).
     */
    array::Array getDisplayColors(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Update the USD extent attribute of all wrapped mesh prims to match their current points.
     */
    void updateExtents();
};

} // namespace objects
} // namespace foundation
} // namespace isaacsim
