// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/limitAPI.h>
#include <pxr/usd/usdPhysics/tokens.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
namespace isaacsim
{
namespace robot
{
namespace schema
{

/**
 * @enum Classes
 * @brief Enumeration of Isaac robot schema API class types.
 * @details
 * Defines the available schema API classes that can be applied to USD prims
 * for robot definition and configuration.
 */
enum class Classes
{
    ROBOT_API,
    LINK_API,
    REFERENCE_POINT_API,
    SITE_API,
    JOINT_API,
    SURFACE_GRIPPER,
    ATTACHMENT_POINT_API
};

/**
 * @enum Attributes
 * @brief Enumeration of Isaac robot schema attribute types.
 * @details
 * Defines the available attribute types that can be set on prims with
 * robot schema APIs applied.
 */
enum class Attributes
{
    DESCRIPTION,
    NAMESPACE,
    ROBOT_TYPE,
    LICENSE,
    VERSION,
    SOURCE,
    CHANGELOG,
    NAME_OVERRIDE,
    REFERENCE_DESCRIPTION,
    FORWARD_AXIS,
    JOINT_NAME_OVERRIDE,
    DOF_OFFSET_OP_ORDER,
    ACTUATOR,
    STATUS,
    RETRY_INTERVAL,
    SHEAR_FORCE_LIMIT,
    COAXIAL_FORCE_LIMIT,
    MAX_GRIP_DISTANCE,
    CLEARANCE_OFFSET,
};

/**
 * @enum Relations
 * @brief Enumeration of Isaac robot schema relationship types.
 * @details
 * Defines the available relationship types that connect prims in the
 * robot schema hierarchy.
 */
enum class Relations
{
    ROBOT_LINKS,
    ROBOT_JOINTS,
    ATTACHMENT_POINTS,
    GRIPPED_OBJECTS
};

/**
 * @enum DofOffsetOpOrder
 * @brief Enumeration of degree-of-freedom offset operation order types.
 * @details
 * Specifies the order of translation and rotation operations for
 * multi-axis joint DOF offsets.
 */
enum class DofOffsetOpOrder
{
    TransX,
    TransY,
    TransZ,
    RotX,
    RotY,
    RotZ
};

/** @brief Common prefix token for Isaac schema attributes. */
const std::string _attrPrefix = "isaac";

/** @brief Array of schema class name strings indexed by Classes enum. */
const std::string classNames[] = { "IsaacRobotAPI", "IsaacLinkAPI",        "IsaacReferencePointAPI", "IsaacSiteAPI",
                                   "IsaacJointAPI", "IsaacSurfaceGripper", "IsaacAttachmentPointAPI" };

/**
 * @brief Get the TfToken for a schema class.
 *
 * @param[in] name The Classes enum value to convert.
 * @return pxr::TfToken Token representing the class name.
 */
inline const pxr::TfToken className(Classes name)
{
    return pxr::TfToken(classNames[static_cast<int>(name)]);
}

namespace custom
{
/**
 * @struct hash
 * @brief Hash function object for enum types.
 * @details
 * Provides a hash function for enum types by casting them to size_t.
 * This allows enums to be used as keys in unordered containers.
 *
 * @tparam E The enum type to provide hashing for
 */
template <typename E>
struct hash
{
    /**
     * @brief Hash function operator for enum values.
     * @details
     * Converts an enum value to its underlying integer representation
     * and casts it to size_t for use as a hash value.
     *
     * @param[in] e The enum value to hash
     * @return size_t Hash value for the enum
     */
    size_t operator()(const E& e) const
    {
        return static_cast<size_t>(e);
    }
};
}

/**
 * @brief Map of attribute enum values to their token names and value types.
 * @details
 * Associates each Attributes enum value with its corresponding USD attribute
 * token name and SDF value type.
 */
const std::unordered_map<Attributes, std::pair<pxr::TfToken, pxr::SdfValueTypeName>> attributeNames = {
    { Attributes::DESCRIPTION, { pxr::TfToken("description"), pxr::SdfValueTypeNames->String } },
    { Attributes::NAMESPACE, { pxr::TfToken("namespace"), pxr::SdfValueTypeNames->String } },
    { Attributes::ROBOT_TYPE, { pxr::TfToken("robotType"), pxr::SdfValueTypeNames->Token } },
    { Attributes::LICENSE, { pxr::TfToken("license"), pxr::SdfValueTypeNames->Token } },
    { Attributes::VERSION, { pxr::TfToken("version"), pxr::SdfValueTypeNames->String } },
    { Attributes::SOURCE, { pxr::TfToken("source"), pxr::SdfValueTypeNames->String } },
    { Attributes::CHANGELOG, { pxr::TfToken("changelog"), pxr::SdfValueTypeNames->StringArray } },
    { Attributes::NAME_OVERRIDE, { pxr::TfToken("nameOverride"), pxr::SdfValueTypeNames->String } },
    { Attributes::REFERENCE_DESCRIPTION, { pxr::TfToken("Description"), pxr::SdfValueTypeNames->String } },
    { Attributes::FORWARD_AXIS, { pxr::TfToken("forwardAxis"), pxr::SdfValueTypeNames->Token } },
    { Attributes::JOINT_NAME_OVERRIDE, { pxr::TfToken("NameOverride"), pxr::SdfValueTypeNames->String } },
    { Attributes::DOF_OFFSET_OP_ORDER, { pxr::TfToken("physics:DofOffsetOpOrder"), pxr::SdfValueTypeNames->TokenArray } },
    { Attributes::ACTUATOR, { pxr::TfToken("actuator"), pxr::SdfValueTypeNames->BoolArray } },
    { Attributes::STATUS, { pxr::TfToken("status"), pxr::SdfValueTypeNames->Token } },
    { Attributes::RETRY_INTERVAL, { pxr::TfToken("retryInterval"), pxr::SdfValueTypeNames->Float } },
    { Attributes::SHEAR_FORCE_LIMIT, { pxr::TfToken("shearForceLimit"), pxr::SdfValueTypeNames->Float } },
    { Attributes::COAXIAL_FORCE_LIMIT, { pxr::TfToken("coaxialForceLimit"), pxr::SdfValueTypeNames->Float } },
    { Attributes::MAX_GRIP_DISTANCE, { pxr::TfToken("maxGripDistance"), pxr::SdfValueTypeNames->Float } },
    { Attributes::CLEARANCE_OFFSET, { pxr::TfToken("clearanceOffset"), pxr::SdfValueTypeNames->Float } }
};

/**
 * @brief Map of relationship enum values to their token names.
 * @details
 * Associates each Relations enum value with its corresponding USD
 * relationship token name.
 */
const std::unordered_map<Relations, pxr::TfToken> relationNames = {
    { Relations::ROBOT_LINKS, pxr::TfToken(_attrPrefix + ":physics:robotLinks") },
    { Relations::ROBOT_JOINTS, pxr::TfToken(_attrPrefix + ":physics:robotJoints") },
    { Relations::ATTACHMENT_POINTS, pxr::TfToken(_attrPrefix + ":attachmentPoints") },
    { Relations::GRIPPED_OBJECTS, pxr::TfToken(_attrPrefix + ":grippedObjects") }
};

/**
 * @brief Get the full attribute name token for an attribute type.
 *
 * @param[in] attr The Attributes enum value.
 * @return pxr::TfToken Full attribute name token with prefix.
 */
inline pxr::TfToken getAttributeName(Attributes attr);

namespace details
{
/** @brief Token for PhysicsD6Joint type name. */
inline const pxr::TfToken kPhysicsD6JointType{ "PhysicsD6Joint" };

/** @brief Token for PhysicsSphericalJoint type name. */
inline const pxr::TfToken kPhysicsSphericalJointType{ "PhysicsSphericalJoint" };

/**
 * @brief Check if a joint prim is a multi-axis joint.
 * @details
 * Returns true if the joint is a D6 joint or a spherical joint,
 * which support multiple degrees of freedom.
 *
 * @param[in] jointPrim The joint prim to check.
 * @return bool True if the joint is multi-axis.
 */
inline bool isMultiAxisJoint(const pxr::UsdPrim& jointPrim)
{
    if (!jointPrim)
    {
        return false;
    }

    const pxr::TfToken typeName = jointPrim.GetTypeName();
    return typeName == kPhysicsD6JointType || typeName == kPhysicsSphericalJointType;
}

/**
 * @brief Descriptor for deprecated DOF (Degree of Freedom) attributes.
 * @details This struct holds information about deprecated joint DOF offset attributes
 * that were used in previous versions of the Isaac physics system.
 */
struct DeprecatedDofAttributeDescriptor
{
    /** @brief The USD attribute name token for the deprecated DOF offset. */
    pxr::TfToken attributeName;
    /** @brief Human-readable name for the DOF axis (e.g., "TransX", "RotY"). */
    std::string tokenName;
    /** @brief The USD physics token representing the axis type. */
    pxr::TfToken axisToken;
};

inline const std::array<DeprecatedDofAttributeDescriptor, 6> kDeprecatedDofAttributes = {
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Tr_X:DoFOffset"), "TransX",
                                      pxr::UsdPhysicsTokens->transX },
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Tr_Y:DoFOffset"), "TransY",
                                      pxr::UsdPhysicsTokens->transY },
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Tr_Z:DoFOffset"), "TransZ",
                                      pxr::UsdPhysicsTokens->transZ },
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Rot_X:DoFOffset"), "RotX", pxr::UsdPhysicsTokens->rotX },
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Rot_Y:DoFOffset"), "RotY", pxr::UsdPhysicsTokens->rotY },
    DeprecatedDofAttributeDescriptor{ pxr::TfToken("isaac:physics:Rot_Z:DoFOffset"), "RotZ", pxr::UsdPhysicsTokens->rotZ }
};

/// @cond DOXYGEN_SHOULD_SKIP_THIS
inline const std::unordered_map<std::string, size_t> kTokenFallbackOrder = []()
{
    std::unordered_map<std::string, size_t> order;
    for (size_t index = 0; index < kDeprecatedDofAttributes.size(); ++index)
    {
        order.emplace(kDeprecatedDofAttributes[index].tokenName, index);
    }
    return order;
}();
/// @endcond

/**
 * @brief Check if a joint axis has valid limit values.
 * @details
 * Verifies that the specified axis has authored low and high limit
 * attributes with the low value less than the high value.
 *
 * @param[in] jointPrim The joint prim to check.
 * @param[in] axisToken The axis token to check limits for.
 * @return bool True if the axis has valid limits.
 */
inline bool axisHasValidLimits(const pxr::UsdPrim& jointPrim, const pxr::TfToken& axisToken)
{
    pxr::UsdPhysicsLimitAPI limitApi = pxr::UsdPhysicsLimitAPI::Get(jointPrim, axisToken);
    if (!limitApi)
    {
        return false;
    }

    pxr::UsdAttribute lowerAttr = limitApi.GetLowAttr();
    pxr::UsdAttribute upperAttr = limitApi.GetHighAttr();
    if (!lowerAttr || !upperAttr || !lowerAttr.HasAuthoredValueOpinion() || !upperAttr.HasAuthoredValueOpinion())
    {
        return false;
    }

    double lower = 0.0;
    double upper = 0.0;
    if (!lowerAttr.Get(&lower) || !upperAttr.Get(&upper))
    {
        return false;
    }

    return lower < upper;
}

/**
 * @brief Compute the fallback order index for a DOF token.
 * @details
 * Looks up the token name in the fallback order map. If not found,
 * returns the provided order index clamped to zero.
 *
 * @param[in] tokenName The DOF token name string.
 * @param[in] orderIndex The order index to use as fallback.
 * @return size_t The computed fallback order.
 */
inline size_t computeFallbackOrder(const std::string& tokenName, int orderIndex)
{
    const auto iterator = kTokenFallbackOrder.find(tokenName);
    if (iterator != kTokenFallbackOrder.end())
    {
        return iterator->second;
    }
    return static_cast<size_t>(std::max(orderIndex, 0));
}

/**
 * @brief Collect deprecated DOF entries from a joint prim.
 * @details
 * Scans the joint prim for deprecated DOF offset attributes and returns
 * them sorted by their offset values and fallback order.
 *
 * @param[in] jointPrim The joint prim to collect entries from.
 * @return std::vector<std::string> Ordered list of DOF token names.
 */
inline std::vector<std::string> collectDeprecatedDofEntries(const pxr::UsdPrim& jointPrim)
{
    std::vector<std::pair<int, std::string>> entries;
    entries.reserve(kDeprecatedDofAttributes.size());

    for (const auto& descriptor : kDeprecatedDofAttributes)
    {
        pxr::UsdAttribute attribute = jointPrim.GetAttribute(descriptor.attributeName);
        if (!attribute || !attribute.HasAuthoredValueOpinion())
        {
            continue;
        }

        if (!axisHasValidLimits(jointPrim, descriptor.axisToken))
        {
            continue;
        }

        int value = 0;
        if (!attribute.Get(&value))
        {
            continue;
        }

        entries.emplace_back(value, descriptor.tokenName);
    }

    if (entries.empty())
    {
        return {};
    }

    std::sort(entries.begin(), entries.end(),
              [](const auto& lhs, const auto& rhs)
              {
                  if (lhs.first != rhs.first)
                  {
                      return lhs.first < rhs.first;
                  }
                  const size_t lhsFallback = computeFallbackOrder(lhs.second, lhs.first);
                  const size_t rhsFallback = computeFallbackOrder(rhs.second, rhs.first);
                  return lhsFallback < rhsFallback;
              });

    std::vector<std::string> orderedTokens;
    orderedTokens.reserve(entries.size());
    for (const auto& entry : entries)
    {
        orderedTokens.push_back(entry.second);
    }

    return orderedTokens;
}

/**
 * @brief Update deprecated joint DOF order attributes.
 * @details
 * Migrates deprecated DOF offset attributes to the new DOF offset op order
 * attribute format for multi-axis joints.
 *
 * @param[in,out] jointPrim The joint prim to update.
 * @return bool True if the joint was updated.
 */
inline bool UpdateDeprecatedJointDofOrder(pxr::UsdPrim& jointPrim)
{
    if (!jointPrim)
    {
        return false;
    }

    if (!details::isMultiAxisJoint(jointPrim))
    {
        return false;
    }

    const std::vector<std::string> orderedTokens = collectDeprecatedDofEntries(jointPrim);
    if (orderedTokens.empty())
    {
        return false;
    }

    const pxr::TfToken attributeToken = getAttributeName(Attributes::DOF_OFFSET_OP_ORDER);
    pxr::UsdAttribute dofAttribute = jointPrim.GetAttribute(attributeToken);
    pxr::VtTokenArray currentValue;

    bool hasCurrentValue = false;
    if (dofAttribute && dofAttribute.HasAuthoredValueOpinion())
    {
        hasCurrentValue = dofAttribute.Get(&currentValue);
        if (hasCurrentValue && currentValue.size() == orderedTokens.size())
        {
            bool identical = true;
            for (size_t index = 0; index < orderedTokens.size(); ++index)
            {
                if (currentValue[index] != pxr::TfToken(orderedTokens[index]))
                {
                    identical = false;
                    break;
                }
            }
            if (identical)
            {
                return false;
            }
        }
    }
    else
    {
        dofAttribute =
            jointPrim.CreateAttribute(attributeToken, attributeNames.at(Attributes::DOF_OFFSET_OP_ORDER).second, false);
    }

    pxr::VtTokenArray newValue(orderedTokens.size());
    for (size_t index = 0; index < orderedTokens.size(); ++index)
    {
        newValue[index] = pxr::TfToken(orderedTokens[index]);
    }

    dofAttribute.Set(newValue);
    return true;
}
}

inline pxr::TfToken getAttributeName(Attributes attr)
{
    return pxr::TfToken(_attrPrefix + ":" + attributeNames.at(attr).first.GetString());
}

inline pxr::SdfPath GetJointBodyRelationship(const pxr::UsdPrim& jointPrim, int bodyIndex);
inline std::pair<pxr::UsdPrim, pxr::UsdPrim> PopulateRobotSchemaFromArticulation(
    const pxr::UsdStagePtr& stage, pxr::UsdPrim& robotPrim, pxr::UsdPrim articulationPrim = pxr::UsdPrim());

/**
 * @brief Apply the IsaacRobotAPI schema to a prim.
 * @details
 * Adds the robot API applied schema and populates the robot schema
 * relationships from any articulation found under the prim.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplyRobotAPI(pxr::UsdPrim& prim)
{
    prim.AddAppliedSchema(pxr::TfToken(classNames[static_cast<int>(Classes::ROBOT_API)]));
    // for (const auto& attr : { Attributes::DESCRIPTION, Attributes::NAMESPACE, Attributes::ROBOT_TYPE,
    //                           Attributes::LICENSE, Attributes::VERSION, Attributes::SOURCE, Attributes::CHANGELOG })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }
    // for (const auto& rel : { Relations::ROBOT_LINKS, Relations::ROBOT_JOINTS })
    // {
    //     prim.CreateRelationship(relationNames.at(rel), false);
    // }

    pxr::UsdStageWeakPtr stageWeak = prim.GetStage();
    pxr::UsdStagePtr stage = stageWeak;
    if (stage)
    {
        PopulateRobotSchemaFromArticulation(stage, prim);
    }
}

/**
 * @brief Apply the IsaacLinkAPI schema to a prim.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplyLinkAPI(pxr::UsdPrim& prim)
{
    prim.AddAppliedSchema(pxr::TfToken(classNames[static_cast<int>(Classes::LINK_API)]));
    // for (const auto& attr : { Attributes::NAME_OVERRIDE })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }
}

/**
 * @brief Apply the IsaacSiteAPI schema to a prim.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplySiteAPI(pxr::UsdPrim& prim)
{
    prim.AddAppliedSchema(pxr::TfToken(classNames[static_cast<int>(Classes::SITE_API)]));
    // for (const auto& attr : { Attributes::REFERENCE_DESCRIPTION, Attributes::FORWARD_AXIS })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }
}

/**
 * @brief Apply the IsaacReferencePointAPI schema to a prim.
 * @deprecated Use ApplySiteAPI instead.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplyReferencePointAPI(pxr::UsdPrim& prim)
{
    fprintf(stderr, "Warning: ApplyReferencePointAPI is deprecated. Use ApplySiteAPI instead.\n");
    ApplySiteAPI(prim);
}

/**
 * @brief Apply the IsaacJointAPI schema to a prim.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplyJointAPI(pxr::UsdPrim& prim)
{
    prim.AddAppliedSchema(pxr::TfToken(classNames[static_cast<int>(Classes::JOINT_API)]));
    // for (const auto& attr :
    //      { Attributes::DOF_OFFSET_OP_ORDER, Attributes::JOINT_NAME_OVERRIDE, Attributes::ACTUATOR })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }
}

/**
 * @brief Create a Surface Gripper prim.
 * @details
 * Creates a new prim with the IsaacSurfaceGripper type at the
 * specified path on the stage.
 *
 * @param[in] stage The USD stage to create the prim in.
 * @param[in] primPath The path where to create the prim.
 * @return pxr::UsdPrim The created Surface Gripper prim.
 */
inline pxr::UsdPrim CreateSurfaceGripper(pxr::UsdStagePtr stage, const std::string& primPath)
{
    // Create the prim
    pxr::UsdPrim prim =
        stage->DefinePrim(pxr::SdfPath(primPath), pxr::TfToken(classNames[static_cast<int>(Classes::SURFACE_GRIPPER)]));

    // Create attributes with default values
    // for (const auto& attr : { Attributes::STATUS, Attributes::SHEAR_FORCE_LIMIT, Attributes::COAXIAL_FORCE_LIMIT,
    //                           Attributes::MAX_GRIP_DISTANCE, Attributes::RETRY_INTERVAL })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }

    // Create relationships
    // for (const auto& rel : { Relations::ATTACHMENT_POINTS, Relations::GRIPPED_OBJECTS })
    // {
    //     prim.CreateRelationship(relationNames.at(rel), false);
    // }

    return prim;
}

/**
 * @brief Apply the IsaacAttachmentPointAPI schema to a prim.
 *
 * @param[in,out] prim The prim to apply the API to.
 */
inline void ApplyAttachmentPointAPI(pxr::UsdPrim& prim)
{
    prim.AddAppliedSchema(pxr::TfToken(classNames[static_cast<int>(Classes::ATTACHMENT_POINT_API)]));
    // for (const auto& attr : { Attributes::FORWARD_AXIS, Attributes::CLEARANCE_OFFSET })
    // {
    //     prim.CreateAttribute(getAttributeName(attr), attributeNames.at(attr).second, false);
    // }
}

/**
 * @brief Obtain the path of a joint body relationship.
 * @details
 * Reads Body0 or Body1 from a joint while respecting the
 * exclude-from-articulation flag.
 *
 * @param[in] jointPrim Joint prim to query.
 * @param[in] bodyIndex Body relationship index (0 or 1).
 *
 * @return pxr::SdfPath Target path or empty path if unavailable.
 */
inline pxr::SdfPath GetJointBodyRelationship(const pxr::UsdPrim& jointPrim, int bodyIndex)
{
    pxr::UsdPhysicsJoint joint(jointPrim);
    if (!joint)
    {
        return pxr::SdfPath();
    }

    bool exclude = false;
    pxr::UsdAttribute excludeAttr = joint.GetExcludeFromArticulationAttr();
    if (excludeAttr && excludeAttr.Get(&exclude) && exclude)
    {
        return pxr::SdfPath();
    }

    pxr::UsdRelationship relation;
    if (bodyIndex == 0)
    {
        relation = joint.GetBody0Rel();
    }
    else if (bodyIndex == 1)
    {
        relation = joint.GetBody1Rel();
    }
    else
    {
        return pxr::SdfPath();
    }

    if (!relation)
    {
        return pxr::SdfPath();
    }

    pxr::SdfPathVector targets;
    relation.GetTargets(&targets);
    if (targets.empty())
    {
        return pxr::SdfPath();
    }

    return targets.front();
}

/**
 * @brief Update deprecated schemas on a robot prim hierarchy.
 * @details
 * Traverses the robot prim and its descendants to migrate deprecated
 * schema APIs to their current equivalents. This includes replacing
 * IsaacReferencePointAPI with IsaacSiteAPI and updating deprecated
 * joint DOF order attributes.
 *
 * @param[in,out] robotPrim The robot prim to update.
 */
inline void UpdateDeprecatedSchemas(pxr::UsdPrim& robotPrim)
{
    if (!robotPrim)
    {
        return;
    }

    pxr::UsdStageWeakPtr stageWeak = robotPrim.GetStage();
    pxr::UsdStagePtr stage = stageWeak;
    if (!stage)
    {
        return;
    }

    const pxr::TfToken referencePointToken(classNames[static_cast<int>(Classes::REFERENCE_POINT_API)]);
    const pxr::TfToken jointToken(classNames[static_cast<int>(Classes::JOINT_API)]);
    for (pxr::UsdPrim prim : pxr::UsdPrimRange(robotPrim))
    {
        if (!prim)
        {
            continue;
        }

        if (prim.HasAPI(referencePointToken))
        {
            prim.RemoveAppliedSchema(referencePointToken);
            ApplySiteAPI(prim);
        }

        if (prim.HasAPI(jointToken))
        {
            details::UpdateDeprecatedJointDofOrder(prim);
        }
    }
}

/**
 * @brief Populate robot schema data from a physics articulation.
 * @details
 * Discovers the articulation root link and root joint, traverses connected
 * rigid bodies through their joints, applies the LinkAPI and JointAPI, and
 * updates the robot relationships with the ordered results.
 *
 * @param[in] stage Stage containing the articulation.
 * @param[in,out] robotPrim Robot prim that stores the relationships.
 * @param[in] articulationPrim Optional prim that has the
 *            UsdPhysicsArticulationRootAPI applied.
 *
 * @return std::pair<pxr::UsdPrim, pxr::UsdPrim> Root link prim and root joint prim.
 */
inline std::pair<pxr::UsdPrim, pxr::UsdPrim> PopulateRobotSchemaFromArticulation(const pxr::UsdStagePtr& stage,
                                                                                 pxr::UsdPrim& robotPrim,
                                                                                 pxr::UsdPrim articulationPrim)
{
    if (!stage || !robotPrim)
    {
        return { pxr::UsdPrim(), pxr::UsdPrim() };
    }

    pxr::UsdPrim articulationRoot = articulationPrim;
    if (!articulationRoot)
    {
        articulationRoot = robotPrim;
    }

    pxr::UsdPhysicsArticulationRootAPI articulationApi(articulationRoot);
    if (!articulationApi)
    {
        for (pxr::UsdPrim prim : pxr::UsdPrimRange(articulationRoot))
        {
            pxr::UsdPhysicsArticulationRootAPI candidate(prim);
            if (candidate)
            {
                articulationRoot = prim;
                articulationApi = candidate;
                break;
            }
        }
    }

    if (!articulationApi)
    {
        return { pxr::UsdPrim(), pxr::UsdPrim() };
    }

    pxr::UsdPrim rootLink = articulationRoot;
    std::vector<pxr::UsdPrim> articulationJoints;
    for (pxr::UsdPrim prim : pxr::UsdPrimRange(articulationRoot))
    {
        if (pxr::UsdPhysicsJoint(prim))
        {
            articulationJoints.push_back(prim);
        }
    }

    pxr::UsdPrim rootJoint;
    if (pxr::UsdPhysicsJoint(articulationRoot))
    {
        rootJoint = articulationRoot;
        pxr::SdfPath bodyPath = GetJointBodyRelationship(rootJoint, 0);
        if (bodyPath.IsEmpty())
        {
            bodyPath = GetJointBodyRelationship(rootJoint, 1);
        }
        if (!bodyPath.IsEmpty())
        {
            pxr::UsdPrim candidateLink = stage->GetPrimAtPath(bodyPath);
            if (candidateLink)
            {
                rootLink = candidateLink;
            }
        }
    }

    if (!rootLink)
    {
        return { pxr::UsdPrim(), rootJoint };
    }

    const std::string rootPath = rootLink.GetPath().GetString();
    std::unordered_map<std::string, std::vector<std::pair<pxr::UsdPrim, int>>> bodyToJoints;
    for (const auto& jointPrim : articulationJoints)
    {
        for (int bodyIndex = 0; bodyIndex < 2; ++bodyIndex)
        {
            pxr::SdfPath bodyPath = GetJointBodyRelationship(jointPrim, bodyIndex);
            if (bodyPath.IsEmpty())
            {
                continue;
            }
            const std::string key = bodyPath.GetString();
            bodyToJoints[key].emplace_back(jointPrim, bodyIndex);
            if (!rootJoint && !rootPath.empty() && key == rootPath)
            {
                rootJoint = jointPrim;
            }
        }
    }

    std::deque<pxr::UsdPrim> queue;
    std::unordered_set<std::string> visitedLinks;
    std::unordered_set<std::string> visitedJoints;
    std::vector<pxr::UsdPrim> orderedLinks;
    std::vector<pxr::UsdPrim> orderedJoints;

    if (rootLink)
    {
        queue.push_back(rootLink);
    }

    while (!queue.empty())
    {
        pxr::UsdPrim linkPrim = queue.front();
        queue.pop_front();
        if (!linkPrim)
        {
            continue;
        }

        const std::string linkKey = linkPrim.GetPath().GetString();
        if (!visitedLinks.insert(linkKey).second)
        {
            continue;
        }

        ApplyLinkAPI(linkPrim);
        orderedLinks.push_back(linkPrim);

        auto iterator = bodyToJoints.find(linkKey);
        if (iterator == bodyToJoints.end())
        {
            continue;
        }

        for (const auto& jointEntry : iterator->second)
        {
            pxr::UsdPrim jointPrim = jointEntry.first;
            if (!jointPrim)
            {
                continue;
            }

            const std::string jointKey = jointPrim.GetPath().GetString();
            if (visitedJoints.insert(jointKey).second)
            {
                ApplyJointAPI(jointPrim);
                orderedJoints.push_back(jointPrim);
            }

            const int otherIndex = jointEntry.second == 0 ? 1 : 0;
            const pxr::SdfPath otherPath = GetJointBodyRelationship(jointPrim, otherIndex);
            if (otherPath.IsEmpty())
            {
                continue;
            }

            const std::string otherKey = otherPath.GetString();
            if (visitedLinks.find(otherKey) != visitedLinks.end())
            {
                continue;
            }

            pxr::UsdPrim otherPrim = stage->GetPrimAtPath(otherPath);
            if (otherPrim)
            {
                queue.push_back(otherPrim);
            }
        }
    }

    pxr::SdfPathVector linkTargets;
    linkTargets.reserve(orderedLinks.size());
    for (const auto& prim : orderedLinks)
    {
        linkTargets.push_back(prim.GetPath());
    }
    pxr::UsdRelationship linkRelationship = robotPrim.CreateRelationship(relationNames.at(Relations::ROBOT_LINKS), true);
    for (const auto& path : linkTargets)
    {
        linkRelationship.AddTarget(path);
    }

    pxr::SdfPathVector jointTargets;
    jointTargets.reserve(orderedJoints.size());
    for (const auto& prim : orderedJoints)
    {
        jointTargets.push_back(prim.GetPath());
    }
    pxr::UsdRelationship jointRelationship =
        robotPrim.CreateRelationship(relationNames.at(Relations::ROBOT_JOINTS), true);
    for (const auto& path : jointTargets)
    {
        jointRelationship.AddTarget(path);
    }

    return { rootLink, rootJoint };
}

}
}
}
