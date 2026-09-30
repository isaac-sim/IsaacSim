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

#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/prims/physics/Articulation.hpp>
#include <isaacsim/foundation/utils/Prim.hpp>

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

namespace isaacsim
{
namespace foundation
{
namespace prims
{
namespace physics
{

namespace
{

constexpr float g_kRadiansToDegrees = 57.29577951308f;
constexpr float g_kDegreesToRadians = 0.01745329252f;

std::string joinPaths(const std::vector<std::string>& v)
{
    std::ostringstream ss;
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            ss << ", ";
        ss << "'" << v[i] << "'";
    }
    return ss.str();
}

// Joint type reported for each USD joint schema type. Types absent from this map are reported as "invalid".
const std::unordered_map<std::string, std::string> g_kJointTypes = {
    { "PhysicsRevoluteJoint", "revolute" },
    { "PhysicsPrismaticJoint", "prismatic" },
    { "PhysicsSphericalJoint", "spherical" },
    { "PhysicsFixedJoint", "fixed" },
};

// DOF type reported for each joint type that contributes a DOF. Joint types absent from this map contribute none.
const std::unordered_map<std::string, std::string> g_kDofTypes = {
    { "revolute", "rotation" },
    { "spherical", "rotation" },
    { "prismatic", "translation" },
};

void checkHomogeneousCount(const std::string& label,
                           std::size_t expectedCount,
                           std::size_t actualCount,
                           const std::string& path)
{
    if (expectedCount != actualCount)
    {
        throw std::runtime_error("Non-homogeneous articulations are not supported: prim '" + path + "' has " +
                                 std::to_string(actualCount) + " " + label + ", but the first prim has " +
                                 std::to_string(expectedCount));
    }
}

std::vector<int32_t> resolveNameIndices(const std::variant<std::string, std::vector<std::string>>& names,
                                        const std::vector<std::string>& referenceNames,
                                        const std::string& label)
{
    // Index the reference names once so that resolving N names costs O(N) lookups rather than O(N) linear scans.
    // Duplicated names resolve to their first occurrence.
    std::unordered_map<std::string_view, int32_t> indexByName;
    indexByName.reserve(referenceNames.size());
    for (std::size_t i = 0; i < referenceNames.size(); ++i)
    {
        indexByName.emplace(referenceNames[i], static_cast<int32_t>(i));
    }

    const std::vector<std::string> requestedNames = std::holds_alternative<std::string>(names) ?
                                                        std::vector<std::string>{ std::get<std::string>(names) } :
                                                        std::get<std::vector<std::string>>(names);
    std::vector<int32_t> indices;
    indices.reserve(requestedNames.size());
    for (const auto& name : requestedNames)
    {
        auto it = indexByName.find(name);
        if (it == indexByName.end())
        {
            throw std::invalid_argument("Unknown " + label + " name: '" + name + "'");
        }
        indices.push_back(it->second);
    }
    return indices;
}

std::string driveAxis(const std::string& dofType)
{
    if (dofType == "rotation")
    {
        return "angular";
    }
    if (dofType == "translation")
    {
        return "linear";
    }
    throw std::runtime_error("Unsupported DOF type: '" + dofType + "'");
}

// Expand an optional index array into concrete, bounds-checked positions. Negative indices count from the end.
// A missing array selects every position in order.
std::vector<int64_t> resolveIndices(const std::optional<array::Array>& indices, int64_t count, const std::string& label)
{
    if (!indices.has_value())
    {
        std::vector<int64_t> allIndices(static_cast<std::size_t>(count));
        std::iota(allIndices.begin(), allIndices.end(), int64_t{ 0 });
        return allIndices;
    }
    std::vector<int64_t> resolved = indices->flatten().get<std::vector<int64_t>>();
    for (int64_t& index : resolved)
    {
        const int64_t original = index;
        if (index < 0)
        {
            index += count;
        }
        if (index < 0 || index >= count)
        {
            throw std::out_of_range(label + " index " + std::to_string(original) + " is out of range [0, " +
                                    std::to_string(count) + ")");
        }
    }
    return resolved;
}

// Batch the prims sitting at `elementIndex` of every selected articulation into a single wrapper, so that one
// attribute round-trip covers the whole column instead of one per (articulation, element) pair.
isaacsim::foundation::objects::Prim elementColumn(const std::vector<std::vector<std::string>>& elementPaths,
                                                  const std::vector<int64_t>& resolvedPrimIndices,
                                                  int64_t elementIndex)
{
    std::vector<std::string> columnPaths;
    columnPaths.reserve(resolvedPrimIndices.size());
    for (int64_t primIndex : resolvedPrimIndices)
    {
        columnPaths.push_back(elementPaths[static_cast<std::size_t>(primIndex)][static_cast<std::size_t>(elementIndex)]);
    }
    return isaacsim::foundation::objects::Prim(columnPaths, /*resolvePaths=*/false);
}

std::vector<float> readColumn(const isaacsim::foundation::objects::Prim& column, const std::string& attributeName)
{
    return std::get<array::Array>(column.getAttributeValues(attributeName)).flatten().get<std::vector<float>>();
}

void writeColumn(const isaacsim::foundation::objects::Prim& column,
                 const std::string& attributeName,
                 const std::vector<float>& values)
{
    column.setAttributeValues(attributeName, array::Array(values));
}

// Assemble a row-major (rowCount, columnCount) array by reading one column at a time. `read(columnPosition)` returns
// one value per selected articulation, or an empty vector to leave the column zero-filled.
template <typename ReadColumn>
array::Array gatherColumns(int64_t rowCount, int64_t columnCount, ReadColumn read)
{
    std::vector<float> values(static_cast<std::size_t>(rowCount * columnCount), 0.0f);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const std::vector<float> column = read(j);
        for (int64_t i = 0; i < rowCount && !column.empty(); ++i)
        {
            values[static_cast<std::size_t>(i * columnCount + j)] = column[static_cast<std::size_t>(i)];
        }
    }
    return array::Array(values).reshape(array::Shape({ rowCount, columnCount }));
}

// Broadcast an input to a dense row-major (rowCount, columnCount) float buffer, following NumPy broadcast rules.
std::vector<float> broadcastMatrix(const array::Array& values, int64_t rowCount, int64_t columnCount)
{
    return values.broadcastTo(array::Shape({ rowCount, columnCount })).flatten().get<std::vector<float>>();
}

std::vector<float> matrixColumn(const std::vector<float>& values, int64_t rowCount, int64_t columnCount, int64_t j)
{
    std::vector<float> column(static_cast<std::size_t>(rowCount));
    for (int64_t i = 0; i < rowCount; ++i)
    {
        column[static_cast<std::size_t>(i)] = values[static_cast<std::size_t>(i * columnCount + j)];
    }
    return column;
}

} // namespace

Articulation::Articulation(const std::variant<std::string, std::vector<std::string>>& paths, bool resetXformOpProperties)
    : isaacsim::foundation::objects::Xform()
{
    // Get prims.
    auto [existentPaths, nonexistentPaths] = this->resolvePaths(paths);
    if (!nonexistentPaths.empty())
    {
        throw std::runtime_error("Specified paths must correspond to existing prims: " + joinPaths(nonexistentPaths));
    }
    m_paths = std::move(existentPaths);
    // Initialize instance from arguments.
    _initialize(resetXformOpProperties);
    _fetchRootPaths();
    _parseMetadata();
}

void Articulation::_fetchRootPaths()
{
    m_rootPaths.clear();
    m_rootPaths.reserve(m_paths.size());
    for (const auto& path : m_paths)
    {
        auto result = isaacsim::foundation::utils::getFirstMatchingChildPrim(
            path,
            [](const std::string& candidatePath) -> bool
            {
                isaacsim::foundation::objects::Prim prim(candidatePath, /*resolvePaths=*/false);
                return prim.hasApi("PhysicsArticulationRootAPI").item<bool>();
            },
            /*includeSelf=*/true);
        if (!result.has_value())
        {
            throw std::runtime_error("No articulation root found for path: " + path);
        }
        m_rootPaths.push_back(*result);
    }
}

void Articulation::_parseMetadata()
{
    std::vector<std::vector<std::string>> dofPaths(m_paths.size());
    std::vector<std::vector<std::string>> jointPaths(m_paths.size());
    std::vector<std::vector<std::string>> linkPaths(m_paths.size());
    std::vector<std::string> dofNames, dofTypes, jointNames, jointTypes, linkNames;

    for (std::size_t i = 0; i < m_paths.size(); ++i)
    {
        // TODO: scope the search by walking the `physics:body0`/`physics:body1` joint relationships once the USD
        //       backends expose relationship targets. Namespace-based traversal mis-scopes articulations whose
        //       members are not confined to a single subtree, and orders links and joints by USD namespace rather
        //       than by the physics engine's internal ordering.
        // TODO: identify mimic joints
        // TODO: identify DOFs via PhysicsDriveAPI
        // TODO: attend to physics:excludeFromArticulation attribute
        std::vector<std::string> paths = isaacsim::foundation::utils::getAllMatchingChildPrims(
            m_paths[i], [](const std::string&) { return true; }, /*includeSelf=*/true);
        isaacsim::foundation::objects::Prim prims(paths, /*resolvePaths=*/false);
        std::vector<bool> areLinks = prims.hasApi("PhysicsRigidBodyAPI").flatten().get<std::vector<bool>>();
        std::vector<bool> areJoints = prims.isA("PhysicsJoint").flatten().get<std::vector<bool>>();
        std::vector<std::string> names = prims.getName();
        std::vector<std::string> typeNames = prims.getTypeName();

        std::vector<std::string> primDofNames, primDofTypes, primJointNames, primJointTypes, primLinkNames;
        for (std::size_t j = 0; j < paths.size(); ++j)
        {
            // Parse links
            if (areLinks[j])
            {
                linkPaths[i].push_back(paths[j]);
                primLinkNames.push_back(names[j]);
            }
            // Parse joints
            if (!areJoints[j])
            {
                continue;
            }
            auto jointTypeIterator = g_kJointTypes.find(typeNames[j]);
            const std::string jointType =
                jointTypeIterator == g_kJointTypes.end() ? "invalid" : jointTypeIterator->second;
            jointPaths[i].push_back(paths[j]);
            primJointNames.push_back(names[j]);
            primJointTypes.push_back(jointType);
            // Parse DOFs (only joints that map to a single rotational or translational axis contribute a DOF).
            // TODO: report the unlocked axes of generic (D6) joints as DOFs.
            auto dofTypeIterator = g_kDofTypes.find(jointType);
            if (dofTypeIterator != g_kDofTypes.end())
            {
                dofPaths[i].push_back(paths[j]);
                primDofNames.push_back(names[j]);
                primDofTypes.push_back(dofTypeIterator->second);
            }
        }

        // Only homogeneous articulations are supported (names and types are shared across all wrapped prims).
        if (i)
        {
            checkHomogeneousCount("DOFs", dofNames.size(), primDofNames.size(), m_paths[i]);
            checkHomogeneousCount("joints", jointNames.size(), primJointNames.size(), m_paths[i]);
            checkHomogeneousCount("links", linkNames.size(), primLinkNames.size(), m_paths[i]);
        }
        else
        {
            linkNames = std::move(primLinkNames);
            jointNames = std::move(primJointNames);
            jointTypes = std::move(primJointTypes);
            dofNames = std::move(primDofNames);
            dofTypes = std::move(primDofTypes);
        }
    }

    m_numDofs = static_cast<int>(dofNames.size());
    m_dofNames = std::move(dofNames);
    m_dofPaths = std::move(dofPaths);
    m_dofTypes = std::move(dofTypes);

    m_numJoints = static_cast<int>(jointNames.size());
    m_jointNames = std::move(jointNames);
    m_jointPaths = std::move(jointPaths);
    m_jointTypes = std::move(jointTypes);

    m_numLinks = static_cast<int>(linkNames.size());
    m_linkNames = std::move(linkNames);
    m_linkPaths = std::move(linkPaths);
}

// -- Articulation metadata --

std::vector<std::string> Articulation::rootPaths() const
{
    return m_rootPaths;
}

int Articulation::numDofs() const
{
    return m_numDofs;
}

std::vector<std::string> Articulation::dofNames() const
{
    return m_dofNames;
}

std::vector<std::vector<std::string>> Articulation::dofPaths() const
{
    return m_dofPaths;
}

std::vector<std::string> Articulation::dofTypes() const
{
    return m_dofTypes;
}

int Articulation::numJoints() const
{
    return m_numJoints;
}

std::vector<std::string> Articulation::jointNames() const
{
    return m_jointNames;
}

std::vector<std::vector<std::string>> Articulation::jointPaths() const
{
    return m_jointPaths;
}

std::vector<std::string> Articulation::jointTypes() const
{
    return m_jointTypes;
}

int Articulation::numLinks() const
{
    return m_numLinks;
}

std::vector<std::string> Articulation::linkNames() const
{
    return m_linkNames;
}

std::vector<std::vector<std::string>> Articulation::linkPaths() const
{
    return m_linkPaths;
}

array::Array Articulation::getDofIndices(const std::variant<std::string, std::vector<std::string>>& names) const
{
    return array::Array(resolveNameIndices(names, m_dofNames, "DOF"));
}

array::Array Articulation::getJointIndices(const std::variant<std::string, std::vector<std::string>>& names) const
{
    return array::Array(resolveNameIndices(names, m_jointNames, "joint"));
}

array::Array Articulation::getLinkIndices(const std::variant<std::string, std::vector<std::string>>& names) const
{
    return array::Array(resolveNameIndices(names, m_linkNames, "link"));
}

// -- DOF-related methods --

std::tuple<array::Array, array::Array> Articulation::getDofLimits(const std::optional<array::Array>& indices,
                                                                  const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    auto readLimit = [&](const std::string& attributeName)
    {
        return gatherColumns(rowCount, columnCount,
                             [&](int64_t j)
                             {
                                 const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                                 const bool isAngular = m_dofTypes[static_cast<std::size_t>(dofIndex)] == "rotation";
                                 std::vector<float> column =
                                     readColumn(elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex), attributeName);
                                 if (isAngular)
                                 {
                                     for (float& value : column)
                                     {
                                         value *= g_kDegreesToRadians;
                                     }
                                 }
                                 return column;
                             });
    };
    return { readLimit("physics:lowerLimit"), readLimit("physics:upperLimit") };
}

void Articulation::setDofLimits(const std::optional<array::Array>& lower,
                                const std::optional<array::Array>& upper,
                                const std::optional<array::Array>& indices,
                                const std::optional<array::Array>& dofIndices)
{
    if (!lower.has_value() && !upper.has_value())
    {
        throw std::invalid_argument("Both 'lower' and 'upper' are not defined. Define at least one of them");
    }
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    std::vector<float> lowerValues, upperValues;
    if (lower.has_value())
    {
        lowerValues = broadcastMatrix(*lower, rowCount, columnCount);
    }
    if (upper.has_value())
    {
        upperValues = broadcastMatrix(*upper, rowCount, columnCount);
    }
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const bool isAngular = m_dofTypes[static_cast<std::size_t>(dofIndex)] == "rotation";
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        auto writeLimit = [&](const std::string& attributeName, const std::vector<float>& source)
        {
            std::vector<float> values = matrixColumn(source, rowCount, columnCount, j);
            if (isAngular)
            {
                for (float& value : values)
                {
                    value *= g_kRadiansToDegrees;
                }
            }
            writeColumn(column, attributeName, values);
        };
        if (lower.has_value())
        {
            writeLimit("physics:lowerLimit", lowerValues);
        }
        if (upper.has_value())
        {
            writeLimit("physics:upperLimit", upperValues);
        }
    }
}

std::tuple<array::Array, array::Array, array::Array> Articulation::getDofFrictionProperties(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    auto readFriction = [&](const std::string& property, bool convertFromDegrees)
    {
        return gatherColumns(rowCount, columnCount,
                             [&](int64_t j)
                             {
                                 const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                                 const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
                                 const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                                 column.applyApi("PhysxJointAxisAPI", axis);
                                 std::vector<float> values =
                                     readColumn(column, "physxJointAxis:" + axis + ":" + property);
                                 if (convertFromDegrees && axis == "angular")
                                 {
                                     for (float& value : values)
                                     {
                                         value *= g_kRadiansToDegrees;
                                     }
                                 }
                                 return values;
                             });
    };
    return { readFriction("staticFrictionEffort", false), readFriction("dynamicFrictionEffort", false),
             readFriction("viscousFrictionCoefficient", true) };
}

void Articulation::setDofFrictionProperties(const std::optional<array::Array>& staticFrictions,
                                            const std::optional<array::Array>& dynamicFrictions,
                                            const std::optional<array::Array>& viscousFrictions,
                                            const std::optional<array::Array>& indices,
                                            const std::optional<array::Array>& dofIndices)
{
    if (!staticFrictions.has_value() && !dynamicFrictions.has_value() && !viscousFrictions.has_value())
    {
        throw std::invalid_argument(
            "All 'staticFrictions', 'dynamicFrictions', and 'viscousFrictions' are not defined. "
            "Define at least one of them");
    }
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    std::vector<float> staticValues, dynamicValues, viscousValues;
    if (staticFrictions.has_value())
    {
        staticValues = broadcastMatrix(*staticFrictions, rowCount, columnCount);
    }
    if (dynamicFrictions.has_value())
    {
        dynamicValues = broadcastMatrix(*dynamicFrictions, rowCount, columnCount);
    }
    if (viscousFrictions.has_value())
    {
        viscousValues = broadcastMatrix(*viscousFrictions, rowCount, columnCount);
    }
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysxJointAxisAPI", axis);
        if (staticFrictions.has_value())
        {
            writeColumn(column, "physxJointAxis:" + axis + ":staticFrictionEffort",
                        matrixColumn(staticValues, rowCount, columnCount, j));
        }
        if (dynamicFrictions.has_value())
        {
            writeColumn(column, "physxJointAxis:" + axis + ":dynamicFrictionEffort",
                        matrixColumn(dynamicValues, rowCount, columnCount, j));
        }
        if (viscousFrictions.has_value())
        {
            std::vector<float> values = matrixColumn(viscousValues, rowCount, columnCount, j);
            if (axis == "angular")
            {
                for (float& value : values)
                {
                    value *= g_kDegreesToRadians;
                }
            }
            writeColumn(column, "physxJointAxis:" + axis + ":viscousFrictionCoefficient", values);
        }
    }
}

std::tuple<array::Array, array::Array, array::Array> Articulation::getDofDriveModelProperties(
    const std::optional<array::Array>& indices, const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    auto readEnvelope = [&](const std::string& property, float angularScale)
    {
        return gatherColumns(
            rowCount, columnCount,
            [&](int64_t j) -> std::vector<float>
            {
                const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
                const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                // DOFs without the performance envelope applied report zero rather than having the API forced onto
                // them, so that querying properties never mutates the stage.
                const std::vector<bool> applied =
                    column.hasApi("PhysxDrivePerformanceEnvelopeAPI", axis).flatten().get<std::vector<bool>>();
                if (std::none_of(applied.begin(), applied.end(), [](bool value) { return value; }))
                {
                    return {};
                }
                std::vector<float> values = readColumn(column, "physxDrivePerformanceEnvelope:" + axis + ":" + property);
                for (std::size_t i = 0; i < values.size(); ++i)
                {
                    values[i] = applied[i] ? values[i] * (axis == "angular" ? angularScale : 1.0f) : 0.0f;
                }
                return values;
            });
    };
    return { readEnvelope("speedEffortGradient", g_kDegreesToRadians),
             readEnvelope("maxActuatorVelocity", g_kDegreesToRadians),
             readEnvelope("velocityDependentResistance", g_kRadiansToDegrees) };
}

void Articulation::setDofDriveModelProperties(const std::optional<array::Array>& speedEffortGradients,
                                              const std::optional<array::Array>& maximumActuatorVelocities,
                                              const std::optional<array::Array>& velocityDependentResistances,
                                              const std::optional<array::Array>& indices,
                                              const std::optional<array::Array>& dofIndices)
{
    if (!speedEffortGradients.has_value() && !maximumActuatorVelocities.has_value() &&
        !velocityDependentResistances.has_value())
    {
        throw std::invalid_argument(
            "All 'speedEffortGradients', 'maximumActuatorVelocities', and 'velocityDependentResistances' are not "
            "defined. Define at least one of them");
    }
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    std::vector<float> gradientValues, velocityValues, resistanceValues;
    if (speedEffortGradients.has_value())
    {
        gradientValues = broadcastMatrix(*speedEffortGradients, rowCount, columnCount);
    }
    if (maximumActuatorVelocities.has_value())
    {
        velocityValues = broadcastMatrix(*maximumActuatorVelocities, rowCount, columnCount);
    }
    if (velocityDependentResistances.has_value())
    {
        resistanceValues = broadcastMatrix(*velocityDependentResistances, rowCount, columnCount);
    }
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysxDrivePerformanceEnvelopeAPI", axis);
        auto writeEnvelope = [&](const std::string& property, const std::vector<float>& source, float angularScale)
        {
            std::vector<float> values = matrixColumn(source, rowCount, columnCount, j);
            if (axis == "angular")
            {
                for (float& value : values)
                {
                    value *= angularScale;
                }
            }
            writeColumn(column, "physxDrivePerformanceEnvelope:" + axis + ":" + property, values);
        };
        if (speedEffortGradients.has_value())
        {
            writeEnvelope("speedEffortGradient", gradientValues, g_kRadiansToDegrees);
        }
        if (maximumActuatorVelocities.has_value())
        {
            writeEnvelope("maxActuatorVelocity", velocityValues, g_kRadiansToDegrees);
        }
        if (velocityDependentResistances.has_value())
        {
            writeEnvelope("velocityDependentResistance", resistanceValues, g_kDegreesToRadians);
        }
    }
}

array::Array Articulation::getDofArmatures(const std::optional<array::Array>& indices,
                                           const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    return gatherColumns(
        static_cast<int64_t>(resolvedPrimIndices.size()), static_cast<int64_t>(resolvedDofIndices.size()),
        [&](int64_t j)
        {
            const auto column =
                elementColumn(m_dofPaths, resolvedPrimIndices, resolvedDofIndices[static_cast<std::size_t>(j)]);
            column.applyApi("PhysxJointAPI");
            return readColumn(column, "physxJoint:armature");
        });
}

void Articulation::setDofArmatures(const array::Array& armatures,
                                   const std::optional<array::Array>& indices,
                                   const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());
    const std::vector<float> values = broadcastMatrix(armatures, rowCount, columnCount);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const auto column =
            elementColumn(m_dofPaths, resolvedPrimIndices, resolvedDofIndices[static_cast<std::size_t>(j)]);
        column.applyApi("PhysxJointAPI");
        writeColumn(column, "physxJoint:armature", matrixColumn(values, rowCount, columnCount, j));
    }
}

std::vector<std::vector<std::string>> Articulation::getDofDriveTypes(const std::optional<array::Array>& indices,
                                                                     const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    std::vector<std::vector<std::string>> types(
        resolvedPrimIndices.size(), std::vector<std::string>(resolvedDofIndices.size()));
    for (std::size_t j = 0; j < resolvedDofIndices.size(); ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[j];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysicsDriveAPI", axis);
        const auto values =
            std::get<std::vector<std::string>>(column.getAttributeValues("drive:" + axis + ":physics:type"));
        for (std::size_t i = 0; i < resolvedPrimIndices.size(); ++i)
        {
            types[i][j] = values[i];
        }
    }
    return types;
}

void Articulation::setDofDriveTypes(const std::variant<std::string, std::vector<std::vector<std::string>>>& types,
                                    const std::optional<array::Array>& indices,
                                    const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const std::size_t rowCount = resolvedPrimIndices.size();
    const std::size_t columnCount = resolvedDofIndices.size();

    // Broadcast the requested types to a dense (N, D) table so that a single string, a single row, or a single
    // column can all stand in for the full table.
    std::vector<std::vector<std::string>> table;
    if (std::holds_alternative<std::string>(types))
    {
        table.assign(rowCount, std::vector<std::string>(columnCount, std::get<std::string>(types)));
    }
    else
    {
        const auto& given = std::get<std::vector<std::vector<std::string>>>(types);
        if (given.empty() || (given.size() != 1 && given.size() != rowCount))
        {
            throw std::invalid_argument("Expected 1 or " + std::to_string(rowCount) + " rows of drive types, got " +
                                        std::to_string(given.size()));
        }
        table.resize(rowCount);
        for (std::size_t i = 0; i < rowCount; ++i)
        {
            const auto& row = given[given.size() == 1 ? 0 : i];
            if (row.empty() || (row.size() != 1 && row.size() != columnCount))
            {
                throw std::invalid_argument("Expected 1 or " + std::to_string(columnCount) +
                                            " drive types per row, got " + std::to_string(row.size()));
            }
            table[i] = row.size() == 1 ? std::vector<std::string>(columnCount, row[0]) : row;
        }
    }

    for (std::size_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[j];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysicsDriveAPI", axis);
        std::vector<std::string> values(rowCount);
        for (std::size_t i = 0; i < rowCount; ++i)
        {
            values[i] = table[i][j];
        }
        column.setAttributeValues("drive:" + axis + ":physics:type", values);
    }
}

array::Array Articulation::getDofMaxVelocities(const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    return gatherColumns(static_cast<int64_t>(resolvedPrimIndices.size()),
                         static_cast<int64_t>(resolvedDofIndices.size()),
                         [&](int64_t j)
                         {
                             const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                             const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                             column.applyApi("PhysxJointAPI");
                             std::vector<float> values = readColumn(column, "physxJoint:maxJointVelocity");
                             if (m_dofTypes[static_cast<std::size_t>(dofIndex)] == "rotation")
                             {
                                 for (float& value : values)
                                 {
                                     value *= g_kDegreesToRadians;
                                 }
                             }
                             return values;
                         });
}

void Articulation::setDofMaxVelocities(const array::Array& maxVelocities,
                                       const std::optional<array::Array>& indices,
                                       const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());
    const std::vector<float> source = broadcastMatrix(maxVelocities, rowCount, columnCount);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysxJointAPI");
        std::vector<float> values = matrixColumn(source, rowCount, columnCount, j);
        if (m_dofTypes[static_cast<std::size_t>(dofIndex)] == "rotation")
        {
            for (float& value : values)
            {
                value *= g_kRadiansToDegrees;
            }
        }
        writeColumn(column, "physxJoint:maxJointVelocity", values);
    }
}

array::Array Articulation::getDofMaxEfforts(const std::optional<array::Array>& indices,
                                            const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    return gatherColumns(static_cast<int64_t>(resolvedPrimIndices.size()),
                         static_cast<int64_t>(resolvedDofIndices.size()),
                         [&](int64_t j)
                         {
                             const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                             const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
                             const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                             column.applyApi("PhysicsDriveAPI", axis);
                             return readColumn(column, "drive:" + axis + ":physics:maxForce");
                         });
}

void Articulation::setDofMaxEfforts(const array::Array& maxEfforts,
                                    const std::optional<array::Array>& indices,
                                    const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());
    const std::vector<float> values = broadcastMatrix(maxEfforts, rowCount, columnCount);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysicsDriveAPI", axis);
        writeColumn(column, "drive:" + axis + ":physics:maxForce", matrixColumn(values, rowCount, columnCount, j));
    }
}

std::tuple<array::Array, array::Array> Articulation::getDofGains(const std::optional<array::Array>& indices,
                                                                 const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    // Angular gains are authored per degree in USD; scaling by 180/pi expresses them per radian.
    auto readGain = [&](const std::string& property)
    {
        return gatherColumns(rowCount, columnCount,
                             [&](int64_t j)
                             {
                                 const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                                 const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
                                 const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                                 column.applyApi("PhysicsDriveAPI", axis);
                                 std::vector<float> values = readColumn(column, "drive:" + axis + ":physics:" + property);
                                 if (axis == "angular")
                                 {
                                     for (float& value : values)
                                     {
                                         value *= g_kRadiansToDegrees;
                                     }
                                 }
                                 return values;
                             });
    };
    return { readGain("stiffness"), readGain("damping") };
}

void Articulation::setDofGains(const std::optional<array::Array>& stiffnesses,
                               const std::optional<array::Array>& dampings,
                               const std::optional<array::Array>& indices,
                               const std::optional<array::Array>& dofIndices,
                               bool updateDefaultGains)
{
    if (!stiffnesses.has_value() && !dampings.has_value())
    {
        throw std::invalid_argument("Both 'stiffnesses' and 'dampings' are not defined. Define at least one of them");
    }
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    std::vector<float> stiffnessValues, dampingValues;
    if (stiffnesses.has_value())
    {
        stiffnessValues = broadcastMatrix(*stiffnesses, rowCount, columnCount);
    }
    if (dampings.has_value())
    {
        dampingValues = broadcastMatrix(*dampings, rowCount, columnCount);
    }
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysicsDriveAPI", axis);
        auto writeGain = [&](const std::string& property, const std::vector<float>& source)
        {
            std::vector<float> values = matrixColumn(source, rowCount, columnCount, j);
            if (axis == "angular")
            {
                for (float& value : values)
                {
                    value *= g_kDegreesToRadians;
                }
            }
            writeColumn(column, "drive:" + axis + ":physics:" + property, values);
        };
        if (stiffnesses.has_value())
        {
            writeGain("stiffness", stiffnessValues);
        }
        if (dampings.has_value())
        {
            writeGain("damping", dampingValues);
        }
    }
    if (updateDefaultGains)
    {
        auto [defaultStiffnesses, defaultDampings] = getDofGains();
        m_defaultDofStiffnesses = std::move(defaultStiffnesses);
        m_defaultDofDampings = std::move(defaultDampings);
    }
}

void Articulation::switchDofControlMode(const std::string& mode,
                                        const std::optional<array::Array>& indices,
                                        const std::optional<array::Array>& dofIndices)
{
    if (mode != "position" && mode != "velocity" && mode != "effort")
    {
        throw std::invalid_argument("Invalid control mode: " + mode +
                                    ". Supported modes are: 'position', 'velocity', 'effort'");
    }
    // Capture the authored gains the first time a mode switch happens; they are what "position" mode restores.
    if (!m_defaultDofStiffnesses.has_value() || !m_defaultDofDampings.has_value())
    {
        auto [defaultStiffnesses, defaultDampings] = getDofGains();
        if (!m_defaultDofStiffnesses.has_value())
        {
            m_defaultDofStiffnesses = std::move(defaultStiffnesses);
        }
        if (!m_defaultDofDampings.has_value())
        {
            m_defaultDofDampings = std::move(defaultDampings);
        }
    }
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());

    auto selectDefaults = [&](const array::Array& defaults)
    {
        const std::vector<float> all = defaults.flatten().get<std::vector<float>>();
        std::vector<float> selected(static_cast<std::size_t>(rowCount * columnCount));
        for (int64_t i = 0; i < rowCount; ++i)
        {
            for (int64_t j = 0; j < columnCount; ++j)
            {
                selected[static_cast<std::size_t>(i * columnCount + j)] =
                    all[static_cast<std::size_t>(resolvedPrimIndices[static_cast<std::size_t>(i)] * m_numDofs +
                                                 resolvedDofIndices[static_cast<std::size_t>(j)])];
            }
        }
        return array::Array(selected).reshape(array::Shape({ rowCount, columnCount }));
    };

    const array::Array zeros(std::vector<float>{ 0.0f });
    const array::Array stiffnesses = mode == "position" ? selectDefaults(*m_defaultDofStiffnesses) : zeros;
    const array::Array dampings = mode == "effort" ? zeros : selectDefaults(*m_defaultDofDampings);
    setDofGains(stiffnesses, dampings, indices, dofIndices, /*updateDefaultGains=*/false);
}

// -- DOF targets --

array::Array Articulation::getDofPositionTargets(const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& dofIndices)
{
    return _getDofDriveTargets("targetPosition", indices, dofIndices);
}

void Articulation::setDofPositionTargets(const array::Array& positions,
                                         const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& dofIndices)
{
    _setDofDriveTargets("targetPosition", positions, indices, dofIndices);
}

array::Array Articulation::getDofVelocityTargets(const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& dofIndices)
{
    return _getDofDriveTargets("targetVelocity", indices, dofIndices);
}

void Articulation::setDofVelocityTargets(const array::Array& velocities,
                                         const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& dofIndices)
{
    _setDofDriveTargets("targetVelocity", velocities, indices, dofIndices);
}

array::Array Articulation::_getDofDriveTargets(const std::string& property,
                                               const std::optional<array::Array>& indices,
                                               const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    return gatherColumns(static_cast<int64_t>(resolvedPrimIndices.size()),
                         static_cast<int64_t>(resolvedDofIndices.size()),
                         [&](int64_t j)
                         {
                             const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
                             const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
                             const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
                             column.applyApi("PhysicsDriveAPI", axis);
                             std::vector<float> values = readColumn(column, "drive:" + axis + ":physics:" + property);
                             if (axis == "angular")
                             {
                                 for (float& value : values)
                                 {
                                     value *= g_kDegreesToRadians;
                                 }
                             }
                             return values;
                         });
}

void Articulation::_setDofDriveTargets(const std::string& property,
                                       const array::Array& targets,
                                       const std::optional<array::Array>& indices,
                                       const std::optional<array::Array>& dofIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedDofIndices = resolveIndices(dofIndices, m_numDofs, "DOF");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedDofIndices.size());
    const std::vector<float> source = broadcastMatrix(targets, rowCount, columnCount);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const int64_t dofIndex = resolvedDofIndices[static_cast<std::size_t>(j)];
        const std::string axis = driveAxis(m_dofTypes[static_cast<std::size_t>(dofIndex)]);
        const auto column = elementColumn(m_dofPaths, resolvedPrimIndices, dofIndex);
        column.applyApi("PhysicsDriveAPI", axis);
        std::vector<float> values = matrixColumn(source, rowCount, columnCount, j);
        if (axis == "angular")
        {
            for (float& value : values)
            {
                value *= g_kRadiansToDegrees;
            }
        }
        writeColumn(column, "drive:" + axis + ":physics:" + property, values);
    }
}

// -- Link-related methods --

array::Array Articulation::getLinkMasses(const std::optional<array::Array>& indices,
                                         const std::optional<array::Array>& linkIndices,
                                         bool inverse)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedLinkIndices = resolveIndices(linkIndices, m_numLinks, "Link");
    return gatherColumns(
        static_cast<int64_t>(resolvedPrimIndices.size()), static_cast<int64_t>(resolvedLinkIndices.size()),
        [&](int64_t j)
        {
            const auto column =
                elementColumn(m_linkPaths, resolvedPrimIndices, resolvedLinkIndices[static_cast<std::size_t>(j)]);
            column.applyApi("PhysicsMassAPI");
            std::vector<float> values = readColumn(column, "physics:mass");
            if (inverse)
            {
                for (float& value : values)
                {
                    value = 1.0f / (value + 1e-8f);
                }
            }
            return values;
        });
}

void Articulation::setLinkMasses(const array::Array& masses,
                                 const std::optional<array::Array>& indices,
                                 const std::optional<array::Array>& linkIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedLinkIndices = resolveIndices(linkIndices, m_numLinks, "Link");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedLinkIndices.size());
    const std::vector<float> values = broadcastMatrix(masses, rowCount, columnCount);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const auto column =
            elementColumn(m_linkPaths, resolvedPrimIndices, resolvedLinkIndices[static_cast<std::size_t>(j)]);
        column.applyApi("PhysicsMassAPI");
        writeColumn(column, "physics:mass", matrixColumn(values, rowCount, columnCount, j));
    }
}

array::Array Articulation::getLinkEnabledGravities(const std::optional<array::Array>& indices,
                                                   const std::optional<array::Array>& linkIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedLinkIndices = resolveIndices(linkIndices, m_numLinks, "Link");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedLinkIndices.size());
    std::vector<bool> enabled(static_cast<std::size_t>(rowCount * columnCount), false);
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const auto column =
            elementColumn(m_linkPaths, resolvedPrimIndices, resolvedLinkIndices[static_cast<std::size_t>(j)]);
        column.applyApi("PhysxRigidBodyAPI");
        const auto disabled = std::get<array::Array>(column.getAttributeValues("physxRigidBody:disableGravity"))
                                  .flatten()
                                  .get<std::vector<bool>>();
        for (int64_t i = 0; i < rowCount; ++i)
        {
            enabled[static_cast<std::size_t>(i * columnCount + j)] = !disabled[static_cast<std::size_t>(i)];
        }
    }
    return array::Array(enabled).reshape(array::Shape({ rowCount, columnCount }));
}

void Articulation::setLinkEnabledGravities(const array::Array& enabled,
                                           const std::optional<array::Array>& indices,
                                           const std::optional<array::Array>& linkIndices)
{
    const std::vector<int64_t> resolvedPrimIndices = _resolveIndexValues(indices);
    const std::vector<int64_t> resolvedLinkIndices = resolveIndices(linkIndices, m_numLinks, "Link");
    const auto rowCount = static_cast<int64_t>(resolvedPrimIndices.size());
    const auto columnCount = static_cast<int64_t>(resolvedLinkIndices.size());
    const auto source = enabled.broadcastTo(array::Shape({ rowCount, columnCount })).flatten().get<std::vector<bool>>();
    for (int64_t j = 0; j < columnCount; ++j)
    {
        const auto column =
            elementColumn(m_linkPaths, resolvedPrimIndices, resolvedLinkIndices[static_cast<std::size_t>(j)]);
        column.applyApi("PhysxRigidBodyAPI");
        std::vector<bool> disabled(static_cast<std::size_t>(rowCount));
        for (int64_t i = 0; i < rowCount; ++i)
        {
            disabled[static_cast<std::size_t>(i)] = !source[static_cast<std::size_t>(i * columnCount + j)];
        }
        column.setAttributeValues("physxRigidBody:disableGravity", array::Array(disabled));
    }
}

// -- PhysX-specific methods --

std::tuple<array::Array, array::Array> Articulation::getSolverIterationCounts(const std::optional<array::Array>& indices)
{
    const Prim rootPrims(m_rootPaths, /*resolvePaths=*/false);
    return {
        std::get<array::Array>(rootPrims.getAttributeValues("physxArticulation:solverPositionIterationCount", indices)),
        std::get<array::Array>(rootPrims.getAttributeValues("physxArticulation:solverVelocityIterationCount", indices))
    };
}

void Articulation::setSolverIterationCounts(const std::optional<array::Array>& positionCounts,
                                            const std::optional<array::Array>& velocityCounts,
                                            const std::optional<array::Array>& indices)
{
    if (!positionCounts.has_value() && !velocityCounts.has_value())
    {
        throw std::invalid_argument(
            "Both 'positionCounts' and 'velocityCounts' are not defined. Define at least one of them");
    }
    const Prim rootPrims(m_rootPaths, /*resolvePaths=*/false);
    if (positionCounts.has_value())
    {
        rootPrims.setAttributeValues("physxArticulation:solverPositionIterationCount", *positionCounts, indices);
    }
    if (velocityCounts.has_value())
    {
        rootPrims.setAttributeValues("physxArticulation:solverVelocityIterationCount", *velocityCounts, indices);
    }
}

array::Array Articulation::getStabilizationThresholds(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(
        Prim(m_rootPaths, /*resolvePaths=*/false).getAttributeValues("physxArticulation:stabilizationThreshold", indices));
}

void Articulation::setStabilizationThresholds(const array::Array& thresholds, const std::optional<array::Array>& indices)
{
    Prim(m_rootPaths, /*resolvePaths=*/false)
        .setAttributeValues("physxArticulation:stabilizationThreshold", thresholds, indices);
}

array::Array Articulation::getEnabledSelfCollisions(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(
        Prim(m_rootPaths, /*resolvePaths=*/false).getAttributeValues("physxArticulation:enabledSelfCollisions", indices));
}

void Articulation::setEnabledSelfCollisions(const array::Array& enabled, const std::optional<array::Array>& indices)
{
    Prim(m_rootPaths, /*resolvePaths=*/false).setAttributeValues("physxArticulation:enabledSelfCollisions", enabled, indices);
}

array::Array Articulation::getSleepThresholds(const std::optional<array::Array>& indices)
{
    return std::get<array::Array>(
        Prim(m_rootPaths, /*resolvePaths=*/false).getAttributeValues("physxArticulation:sleepThreshold", indices));
}

void Articulation::setSleepThresholds(const array::Array& thresholds, const std::optional<array::Array>& indices)
{
    Prim(m_rootPaths, /*resolvePaths=*/false).setAttributeValues("physxArticulation:sleepThreshold", thresholds, indices);
}

array::Array Articulation::areOfType(const std::variant<std::string, std::vector<std::string>>& paths)
{
    const isaacsim::foundation::objects::Prim prims(paths);
    const std::vector<bool> areXformable = prims.isA("Xformable").flatten().get<std::vector<bool>>();
    const std::vector<std::string>& primPaths = prims.paths();
    std::vector<bool> result(primPaths.size());
    for (std::size_t i = 0; i < primPaths.size(); ++i)
    {
        // Same search the constructor performs to locate the articulation root, but bounded to the prim itself
        // and its direct children so classification stays cheap for large hierarchies.
        result[i] =
            areXformable[i] && isaacsim::foundation::utils::getFirstMatchingChildPrim(
                                   primPaths[i],
                                   [](const std::string& candidatePath) -> bool
                                   {
                                       isaacsim::foundation::objects::Prim prim(candidatePath, /*resolvePaths=*/false);
                                       return prim.hasApi("PhysicsArticulationRootAPI").item<bool>();
                                   },
                                   /*includeSelf=*/true, /*maxDepth=*/1)
                                   .has_value();
    }
    return array::Array(result).reshape(array::Shape({ -1, 1 }));
}

} // namespace physics
} // namespace prims
} // namespace foundation
} // namespace isaacsim
