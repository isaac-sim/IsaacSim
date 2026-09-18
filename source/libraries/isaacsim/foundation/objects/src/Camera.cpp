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

#include <isaacsim/foundation/objects/Camera.hpp>

#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

namespace
{

// USD stores focal lengths and apertures in tenths of a scene unit; the public API works in scene
// units, so values are scaled by 10 when writing to USD and by 0.1 when reading back.
constexpr double g_kApertureScale = 10.0;

array::Array toScaledColumn(const array::Array& values, int64_t size, double factor)
{
    auto data = values.broadcastTo(array::Shape({ size, int64_t{ 1 } })).get<std::vector<std::vector<double>>>();
    for (auto& row : data)
    {
        for (auto& value : row)
        {
            value *= factor;
        }
    }
    return array::Array(data);
}

array::Array scaled(const array::Array& values, double factor)
{
    auto data = values.flatten().get<std::vector<double>>();
    for (auto& value : data)
    {
        value *= factor;
    }
    return array::Array(data).reshape(array::Shape({ static_cast<int64_t>(values.size()), int64_t{ 1 } }));
}

} // namespace

Camera::Camera(const std::variant<std::string, std::vector<std::string>>& paths, bool resetXformOpProperties) : Xform()
{
    // Get or create camera prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    // Get camera prims.
    if (!existentPaths.empty())
    {
        m_paths = std::move(existentPaths);
        const std::vector<bool> isCamera = this->isA("Camera").flatten().get<std::vector<bool>>();
        for (std::size_t i = 0; i < m_paths.size(); ++i)
        {
            if (!isCamera[i])
            {
                throw std::runtime_error("The wrapped prim at path '" + m_paths[i] + "' is not a USD Camera");
            }
        }
    }
    // Create camera prims.
    else
    {
        m_paths = std::move(nonexistentPaths);
        for (const auto& path : m_paths)
        {
            this->getStage().definePrim(path, "Camera");
        }
    }
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
}


void Camera::setFocalLengths(const array::Array& focalLengths, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("focalLength", toScaledColumn(focalLengths, batchSize, g_kApertureScale), indices);
}

array::Array Camera::getFocalLengths(const std::optional<array::Array>& indices)
{
    return scaled(std::get<array::Array>(this->getAttributeValues("focalLength", indices)), 1.0 / g_kApertureScale);
}

void Camera::setFocusDistances(const array::Array& focusDistances, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues(
        "focusDistance", focusDistances.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Camera::getFocusDistances(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("focusDistance", indices));
}

void Camera::setStereoRoles(const std::variant<std::string, std::vector<std::string>>& roles,
                            const std::optional<array::Array>& indices)
{
    this->setAttributeValues("stereoRole", _resolveStringList(roles, indices), indices);
}

std::vector<std::string> Camera::getStereoRoles(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("stereoRole", indices));
}

void Camera::setFStops(const array::Array& fstops, const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    this->setAttributeValues("fStop", fstops.broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
}

array::Array Camera::getFStops(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(this->getAttributeValues("fStop", indices));
}

void Camera::setApertures(const std::optional<array::Array>& horizontalApertures,
                          const std::optional<array::Array>& verticalApertures,
                          const std::optional<array::Array>& indices)
{
    if (!horizontalApertures.has_value() && !verticalApertures.has_value())
    {
        throw std::invalid_argument(
            "Both 'horizontalApertures' and 'verticalApertures' are not defined. Define at least one of them");
    }
    int64_t batchSize = _resolveIndexedSize(indices);
    if (horizontalApertures.has_value())
    {
        this->setAttributeValues(
            "horizontalAperture", toScaledColumn(*horizontalApertures, batchSize, g_kApertureScale), indices);
    }
    if (verticalApertures.has_value())
    {
        this->setAttributeValues(
            "verticalAperture", toScaledColumn(*verticalApertures, batchSize, g_kApertureScale), indices);
    }
}

std::tuple<array::Array, array::Array> Camera::getApertures(const std::optional<array::Array>& indices)
{
    return {
        scaled(std::get<array::Array>(this->getAttributeValues("horizontalAperture", indices)), 1.0 / g_kApertureScale),
        scaled(std::get<array::Array>(this->getAttributeValues("verticalAperture", indices)), 1.0 / g_kApertureScale)
    };
}

void Camera::setApertureOffsets(const std::optional<array::Array>& horizontalOffsets,
                                const std::optional<array::Array>& verticalOffsets,
                                const std::optional<array::Array>& indices)
{
    if (!horizontalOffsets.has_value() && !verticalOffsets.has_value())
    {
        throw std::invalid_argument(
            "Both 'horizontalOffsets' and 'verticalOffsets' are not defined. Define at least one of them");
    }
    int64_t batchSize = _resolveIndexedSize(indices);
    if (horizontalOffsets.has_value())
    {
        this->setAttributeValues(
            "horizontalApertureOffset", toScaledColumn(*horizontalOffsets, batchSize, g_kApertureScale), indices);
    }
    if (verticalOffsets.has_value())
    {
        this->setAttributeValues(
            "verticalApertureOffset", toScaledColumn(*verticalOffsets, batchSize, g_kApertureScale), indices);
    }
}

std::tuple<array::Array, array::Array> Camera::getApertureOffsets(const std::optional<array::Array>& indices)
{
    return { scaled(std::get<array::Array>(this->getAttributeValues("horizontalApertureOffset", indices)),
                    1.0 / g_kApertureScale),
             scaled(std::get<array::Array>(this->getAttributeValues("verticalApertureOffset", indices)),
                    1.0 / g_kApertureScale) };
}

void Camera::setProjections(const std::variant<std::string, std::vector<std::string>>& projections,
                            const std::optional<array::Array>& indices)
{
    this->setAttributeValues("projection", _resolveStringList(projections, indices), indices);
}

std::vector<std::string> Camera::getProjections(const std::optional<array::Array>& indices)
{
    return std::get<std::vector<std::string>>(this->getAttributeValues("projection", indices));
}

void Camera::setClippingRanges(const std::optional<array::Array>& nearDistances,
                               const std::optional<array::Array>& farDistances,
                               const std::optional<array::Array>& indices)
{
    if (!nearDistances.has_value() && !farDistances.has_value())
    {
        throw std::invalid_argument(
            "Both 'nearDistances' and 'farDistances' are not defined. Define at least one of them");
    }
    // The full clipping range (near, far) is read and re-written for the selected prims, so the
    // provided component(s) are broadcast to match the number of indexed prims.
    auto ranges =
        std::get<array::Array>(this->getAttributeValues("clippingRange", indices)).get<std::vector<std::vector<double>>>();
    const int64_t size = static_cast<int64_t>(ranges.size());
    if (nearDistances.has_value())
    {
        auto nearData =
            (*nearDistances).broadcastTo(array::Shape({ size, int64_t{ 1 } })).get<std::vector<std::vector<double>>>();
        for (std::size_t i = 0; i < ranges.size(); ++i)
        {
            ranges[i][0] = nearData[i][0];
        }
    }
    if (farDistances.has_value())
    {
        auto farData =
            (*farDistances).broadcastTo(array::Shape({ size, int64_t{ 1 } })).get<std::vector<std::vector<double>>>();
        for (std::size_t i = 0; i < ranges.size(); ++i)
        {
            ranges[i][1] = farData[i][0];
        }
    }
    this->setAttributeValues("clippingRange", array::Array(ranges), indices);
}

std::tuple<array::Array, array::Array> Camera::getClippingRanges(const std::optional<array::Array>& indices)
{
    auto ranges =
        std::get<array::Array>(this->getAttributeValues("clippingRange", indices)).get<std::vector<std::vector<double>>>();
    std::vector<double> nearDistances, farDistances;
    nearDistances.reserve(ranges.size());
    farDistances.reserve(ranges.size());
    for (const auto& range : ranges)
    {
        nearDistances.push_back(range[0]);
        farDistances.push_back(range[1]);
    }
    auto size = static_cast<int64_t>(ranges.size());
    return { array::Array(nearDistances).reshape(array::Shape({ size, int64_t{ 1 } })),
             array::Array(farDistances).reshape(array::Shape({ size, int64_t{ 1 } })) };
}

void Camera::setShutterTimes(const std::optional<array::Array>& openTimes,
                             const std::optional<array::Array>& closeTimes,
                             const std::optional<array::Array>& indices)
{
    if (!openTimes.has_value() && !closeTimes.has_value())
    {
        throw std::invalid_argument("Both 'openTimes' and 'closeTimes' are not defined. Define at least one of them");
    }
    int64_t batchSize = _resolveIndexedSize(indices);
    if (openTimes.has_value())
    {
        this->setAttributeValues(
            "shutter:open", (*openTimes).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
    }
    if (closeTimes.has_value())
    {
        this->setAttributeValues(
            "shutter:close", (*closeTimes).broadcastTo(array::Shape({ batchSize, int64_t{ 1 } })), indices);
    }
}

std::tuple<array::Array, array::Array> Camera::getShutterTimes(const std::optional<array::Array>& indices)
{
    return { std::get<array::Array>(this->getAttributeValues("shutter:open", indices)),
             std::get<array::Array>(this->getAttributeValues("shutter:close", indices)) };
}

void Camera::enforceSquarePixels(const array::Array& resolutions,
                                 const std::variant<std::string, std::vector<std::string>>& modes,
                                 const std::optional<array::Array>& indices)
{
    int64_t batchSize = _resolveIndexedSize(indices);
    // Resolutions follow the OpenCV/NumPy convention: (height, width).
    auto resolutionData =
        resolutions.broadcastTo(array::Shape({ batchSize, int64_t{ 2 } })).get<std::vector<std::vector<double>>>();
    const std::vector<std::string> modeData = _resolveStringList(modes, indices);
    auto [horizontalArray, verticalArray] = this->getApertures(indices);
    auto horizontal = horizontalArray.flatten().get<std::vector<double>>();
    auto vertical = verticalArray.flatten().get<std::vector<double>>();
    for (std::size_t i = 0; i < static_cast<std::size_t>(batchSize); ++i)
    {
        double aspectRatio = resolutionData[i][1] / resolutionData[i][0];
        if (modeData[i] == "horizontal")
        {
            vertical[i] = horizontal[i] / aspectRatio;
        }
        else if (modeData[i] == "vertical")
        {
            horizontal[i] = vertical[i] * aspectRatio;
        }
        else
        {
            throw std::invalid_argument("Invalid mode: '" + modeData[i] +
                                        "'. Valid modes are 'horizontal' and 'vertical'");
        }
    }
    // Reshape to (size, 1) columns and re-apply through setApertures (which handles the unit scaling).
    std::vector<std::vector<double>> horizontalColumn, verticalColumn;
    horizontalColumn.reserve(horizontal.size());
    verticalColumn.reserve(vertical.size());
    for (std::size_t i = 0; i < horizontal.size(); ++i)
    {
        horizontalColumn.push_back({ horizontal[i] });
        verticalColumn.push_back({ vertical[i] });
    }
    // Columns are sized to the selected prims and written back through the same indices.
    this->setApertures(array::Array(horizontalColumn), array::Array(verticalColumn), indices);
}

array::Array Camera::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return Prim(paths).isA("Camera");
}

} // namespace objects
} // namespace foundation
} // namespace isaacsim
