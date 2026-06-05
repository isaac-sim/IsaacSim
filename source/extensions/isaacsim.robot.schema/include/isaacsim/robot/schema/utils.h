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

#pragma once

#include "robot_schema.h"

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quaternion.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include <array>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
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
 * @brief Gather all joint prims referenced by a robot prim.
 * @details
 * Traverses relationship targets recursively to collect every prim that
 * applies the joint API. When parseNestedRobots is enabled, the traversal
 * continues even if the target does not apply the joint API, allowing nested
 * robot aggregations to be parsed.
 *
 * @param[in] stage Stage that owns the prims.
 * @param[in] robotLinkPrim Prim that anchors the traversal.
 * @param[in] parseNestedRobots Enables traversal through non-joint prims.
 *
 * @return std::vector<pxr::UsdPrim> Collection of joint prims.
 */
inline std::vector<pxr::UsdPrim> GetAllRobotJoints(const pxr::UsdStagePtr& stage,
                                                   const pxr::UsdPrim& robotLinkPrim,
                                                   bool parseNestedRobots = true)
{
    std::vector<pxr::UsdPrim> joints;
    if (!stage || !robotLinkPrim)
    {
        return joints;
    }

    if (robotLinkPrim.HasAPI(className(Classes::JOINT_API)))
    {
        joints.push_back(robotLinkPrim);
        return joints;
    }

    if (robotLinkPrim.HasAPI(className(Classes::ROBOT_API)))
    {
        pxr::UsdRelationship relation = robotLinkPrim.GetRelationship(relationNames.at(Relations::ROBOT_JOINTS));
        if (relation)
        {
            pxr::SdfPathVector targets;
            relation.GetTargets(&targets);
            for (const auto& jointPath : targets)
            {
                pxr::UsdPrim jointPrim = stage->GetPrimAtPath(jointPath);
                if (!jointPrim)
                {
                    continue;
                }
                if (jointPrim.HasAPI(className(Classes::JOINT_API)) || parseNestedRobots)
                {
                    const auto nested = GetAllRobotJoints(stage, jointPrim, parseNestedRobots);
                    joints.insert(joints.end(), nested.begin(), nested.end());
                }
            }
        }
    }

    return joints;
}

/**
 * @brief Gather all link prims referenced by a robot prim.
 * @details
 * Traverses relationship targets recursively to collect prims that apply the
 * link API. Optionally, reference points are treated as links.
 *
 * @param[in] stage Stage that owns the prims.
 * @param[in] robotLinkPrim Prim that anchors the traversal.
 * @param[in] includeReferencePoints Includes reference point prims as links.
 *
 * @return std::vector<pxr::UsdPrim> Collection of link prims.
 */
inline std::vector<pxr::UsdPrim> GetAllRobotLinks(const pxr::UsdStagePtr& stage,
                                                  const pxr::UsdPrim& robotLinkPrim,
                                                  bool includeReferencePoints = false)
{
    std::vector<pxr::UsdPrim> links;
    if (!stage || !robotLinkPrim)
    {
        return links;
    }

    if (robotLinkPrim.HasAPI(className(Classes::LINK_API)))
    {
        links.push_back(robotLinkPrim);
        return links;
    }

    if (includeReferencePoints && robotLinkPrim.HasAPI(className(Classes::REFERENCE_POINT_API)))
    {
        links.push_back(robotLinkPrim);
        return links;
    }

    if (robotLinkPrim.HasAPI(className(Classes::ROBOT_API)))
    {
        pxr::UsdRelationship relation = robotLinkPrim.GetRelationship(relationNames.at(Relations::ROBOT_LINKS));
        if (relation)
        {
            pxr::SdfPathVector targets;
            relation.GetTargets(&targets);
            for (const auto& linkPath : targets)
            {
                pxr::UsdPrim linkPrim = stage->GetPrimAtPath(linkPath);
                if (!linkPrim)
                {
                    continue;
                }
                const auto nested = GetAllRobotLinks(stage, linkPrim, includeReferencePoints);
                links.insert(links.end(), nested.begin(), nested.end());
            }
        }
    }

    return links;
}

/**
 * @brief Collect every prim in a robot's subtree that applies an Isaac robot-schema API.
 * @details
 * Descends `robotPrim` (depth-first prim-tree order) and returns prims tagged with any of:
 * `IsaacRobotAPI` (nested sub-robots), `IsaacLinkAPI`, `IsaacJointAPI`, `IsaacSiteAPI`,
 * `IsaacReferencePointAPI`. Callers filter by API as needed.
 *
 * This is a pure schema query — it does not assign frame relationships, parent links,
 * or any consumer-specific semantics; consumers reconstruct those from the returned prims.
 *
 * @param[in] stage Stage that owns the prims.
 * @param[in] robotPrim Prim whose subtree to walk (caller typically passes an
 *                      `IsaacRobotAPI`-bearing prim).
 *
 * @return Prims in depth-first descent order. Empty on invalid input or no
 *         schema-tagged descendants.
 */
inline std::vector<pxr::UsdPrim> GetAllRobotComponents(const pxr::UsdStagePtr& stage, const pxr::UsdPrim& robotPrim)
{
    std::vector<pxr::UsdPrim> components;
    if (!stage || !robotPrim)
    {
        return components;
    }
    const std::array<pxr::TfToken, 5> isaacApis = { className(Classes::ROBOT_API), className(Classes::LINK_API),
                                                    className(Classes::JOINT_API), className(Classes::SITE_API),
                                                    className(Classes::REFERENCE_POINT_API) };
    for (const pxr::UsdPrim& descendant : pxr::UsdPrimRange(robotPrim))
    {
        for (const auto& api : isaacApis)
        {
            if (descendant.HasAPI(api))
            {
                components.push_back(descendant);
                break;
            }
        }
    }
    return components;
}

/**
 * @brief Build a child-link to parent-link path map from an `IsaacRobotAPI` prim.
 * @details
 * Walks `isaac:physics:robotJoints` rel targets; for each joint, reads `body0` (parent)
 * and `body1` (child) via `GetJointBodyRelationship` and records `body1 -> body0`.
 * Links not referenced as any joint's `body1` (the articulation-root link) do not appear
 * in the returned map — callers should treat absence as "parents to world / outer scope".
 *
 * @param[in] stage Stage that owns the prims.
 * @param[in] robotPrim The `IsaacRobotAPI`-bearing prim (caller should verify this).
 *
 * @return Map of `child_link_path` -> `parent_link_path`; empty if no robotJoints rel.
 */
inline std::unordered_map<std::string, std::string> GetRobotLinkParentMap(const pxr::UsdStagePtr& stage,
                                                                          const pxr::UsdPrim& robotPrim)
{
    std::unordered_map<std::string, std::string> parentMap;
    if (!stage || !robotPrim)
    {
        return parentMap;
    }
    pxr::UsdRelationship robotJointsRel = robotPrim.GetRelationship(relationNames.at(Relations::ROBOT_JOINTS));
    if (!robotJointsRel)
    {
        return parentMap;
    }
    pxr::SdfPathVector jointPaths;
    robotJointsRel.GetTargets(&jointPaths);
    for (const auto& jointPath : jointPaths)
    {
        pxr::UsdPrim jointPrim = stage->GetPrimAtPath(jointPath);
        if (!jointPrim)
        {
            continue;
        }
        pxr::SdfPath body0Path = GetJointBodyRelationship(jointPrim, 0);
        pxr::SdfPath body1Path = GetJointBodyRelationship(jointPrim, 1);
        if (body0Path.IsEmpty() || body1Path.IsEmpty())
        {
            continue;
        }
        parentMap[body1Path.GetString()] = body0Path.GetString();
    }
    return parentMap;
}

/**
 * @class RobotLinkNode
 * @brief Node representation for the robot kinematic tree.
 * @details
 * Stores the prim metadata, parent relationship, outgoing child links, and
 * the joint that attaches the node to its parent.
 */
class RobotLinkNode
{
public:
    /**
     * @brief Construct a node for a given prim.
     *
     * @param[in] prim Prim represented by this node.
     * @param[in] parent Parent node in the tree.
     * @param[in] joint Joint that connects the parent to this node.
     */
    RobotLinkNode(const pxr::UsdPrim& prim = pxr::UsdPrim(),
                  const std::shared_ptr<RobotLinkNode>& parent = nullptr,
                  const pxr::UsdPrim& joint = pxr::UsdPrim())
        : m_prim(prim), m_parent(parent), m_jointToParent(joint)
    {
        if (m_prim)
        {
            m_path = m_prim.GetPath();
            m_name = m_prim.GetName().GetString();
        }
    }

    /**
     * @brief Add a child node.
     *
     * @param[in] child Node to append.
     */
    void addChild(const std::shared_ptr<RobotLinkNode>& child)
    {
        m_children.push_back(child);
    }

    /**
     * @brief Record a joint connected to this link.
     *
     * @param[in] joint Joint prim to store.
     */
    void addJoint(const pxr::UsdPrim& joint)
    {
        m_joints.push_back(joint);
    }

    /**
     * @brief Access the prim associated with this node.
     *
     * @return const pxr::UsdPrim& Referenced prim.
     */
    const pxr::UsdPrim& getPrim() const
    {
        return m_prim;
    }

    /**
     * @brief Access the prim path.
     *
     * @return const pxr::SdfPath& Referenced path.
     */
    const pxr::SdfPath& getPath() const
    {
        return m_path;
    }

    /**
     * @brief Access the prim name.
     *
     * @return const std::string& Prim name.
     */
    const std::string& getName() const
    {
        return m_name;
    }

    /**
     * @brief Get the parent node.
     *
     * @return std::shared_ptr<RobotLinkNode> Parent node, if any.
     */
    std::shared_ptr<RobotLinkNode> getParent() const
    {
        return m_parent.lock();
    }

    /**
     * @brief Access the children list.
     *
     * @return const std::vector<std::shared_ptr<RobotLinkNode>>& Child nodes.
     */
    const std::vector<std::shared_ptr<RobotLinkNode>>& getChildren() const
    {
        return m_children;
    }

    /**
     * @brief Access the joints connected at this node.
     *
     * @return const std::vector<pxr::UsdPrim>& Stored joints.
     */
    const std::vector<pxr::UsdPrim>& getJoints() const
    {
        return m_joints;
    }

    /**
     * @brief Get the joint that connects this node to its parent.
     *
     * @return const pxr::UsdPrim& Joint prim.
     */
    const pxr::UsdPrim& getJointToParent() const
    {
        return m_jointToParent;
    }

private:
    pxr::UsdPrim m_prim;
    pxr::SdfPath m_path;
    std::string m_name;
    std::weak_ptr<RobotLinkNode> m_parent;
    std::vector<std::shared_ptr<RobotLinkNode>> m_children;
    std::vector<pxr::UsdPrim> m_joints;
    pxr::UsdPrim m_jointToParent;
};

/**
 * @brief Print the link tree to stdout.
 * @details
 * Recursively prints the node names, indenting two spaces per level.
 *
 * @param[in] root Root node to print.
 * @param[in] indent Current indentation width.
 */
inline void PrintRobotTree(const std::shared_ptr<RobotLinkNode>& root, size_t indent = 0)
{
    if (!root)
    {
        return;
    }

    std::cout << std::string(indent, ' ') << root->getName() << std::endl;
    for (const auto& child : root->getChildren())
    {
        PrintRobotTree(child, indent + 2);
    }
}

/**
 * @brief Compute a joint pose relative to the robot prim.
 * @details
 * Uses the joint attributes and body transforms to compute the joint matrix
 * expressed in the robot coordinate system.
 *
 * @param[in] robotPrim Robot prim used as the frame of reference.
 * @param[in] jointPrim Joint prim to evaluate.
 *
 * @return std::optional<pxr::GfMatrix4d> Pose matrix when available.
 */
inline std::optional<pxr::GfMatrix4d> GetJointPose(const pxr::UsdPrim& robotPrim, const pxr::UsdPrim& jointPrim)
{
    if (!robotPrim || !jointPrim)
    {
        return std::nullopt;
    }

    pxr::UsdPhysicsJoint joint(jointPrim);
    if (!joint)
    {
        return std::nullopt;
    }

    pxr::UsdStageWeakPtr stageWeak = jointPrim.GetStage();
    pxr::UsdStagePtr stage = stageWeak;
    if (!stage)
    {
        return std::nullopt;
    }

    pxr::UsdGeomXformCache cache(pxr::UsdTimeCode::Default());
    const pxr::GfMatrix4d robotTransform = cache.GetLocalToWorldTransform(robotPrim);
    const pxr::GfMatrix4d robotInverse = robotTransform.GetInverse();

    auto computePose = [&](int bodyIndex, const pxr::GfVec3f& localPos, const pxr::GfQuatf& localRot,
                           const pxr::UsdPrim& bodyPrim) -> pxr::GfMatrix4d
    {
        pxr::GfMatrix4d localMatrix(1.0);
        localMatrix.SetTranslate(pxr::GfVec3d(localPos));
        localMatrix.SetRotateOnly(pxr::GfQuatd(localRot));
        const pxr::GfMatrix4d bodyPose = cache.GetLocalToWorldTransform(bodyPrim);
        if (bodyIndex == 0)
        {
            return localMatrix * bodyPose;
        }
        return bodyPose * localMatrix;
    };

    const pxr::SdfPath body0Path = GetJointBodyRelationship(jointPrim, 0);
    if (!body0Path.IsEmpty())
    {
        pxr::UsdPrim body0Prim = stage->GetPrimAtPath(body0Path);
        if (body0Prim)
        {
            pxr::GfVec3f translate(0.0f);
            joint.GetLocalPos0Attr().Get(&translate);
            pxr::GfQuatf rotation(1.0f);
            joint.GetLocalRot0Attr().Get(&rotation);
            const pxr::GfMatrix4d jointPose = computePose(0, translate, rotation, body0Prim);
            return jointPose * robotInverse;
        }
    }

    const pxr::SdfPath body1Path = GetJointBodyRelationship(jointPrim, 1);
    if (!body1Path.IsEmpty())
    {
        pxr::UsdPrim body1Prim = stage->GetPrimAtPath(body1Path);
        if (body1Prim)
        {
            pxr::GfVec3f translate(0.0f);
            joint.GetLocalPos1Attr().Get(&translate);
            pxr::GfQuatf rotation(1.0f);
            joint.GetLocalRot1Attr().Get(&rotation);
            const pxr::GfMatrix4d jointPose = computePose(1, translate, rotation, body1Prim);
            return jointPose * robotInverse;
        }
    }

    return std::nullopt;
}

/**
 * @brief Split links before and after a joint.
 * @details
 * Traverses the tree rooted at the provided node to determine which links
 * precede and follow the joint along the kinematic chain.
 *
 * @param[in] root Root of the robot tree.
 * @param[in] jointPrim Joint to evaluate.
 *
 * @return std::pair<std::vector<pxr::UsdPrim>, std::vector<pxr::UsdPrim>> Pair of link lists.
 */
inline std::pair<std::vector<pxr::UsdPrim>, std::vector<pxr::UsdPrim>> GetLinksFromJoint(
    const std::shared_ptr<RobotLinkNode>& root, const pxr::UsdPrim& jointPrim)
{
    if (!root || !jointPrim)
    {
        return { {}, {} };
    }

    std::function<std::shared_ptr<RobotLinkNode>(const std::shared_ptr<RobotLinkNode>&)> findNodeWithJoint =
        [&](const std::shared_ptr<RobotLinkNode>& node) -> std::shared_ptr<RobotLinkNode>
    {
        if (!node)
        {
            return nullptr;
        }
        if (node->getJointToParent() == jointPrim)
        {
            return node;
        }
        for (const auto& child : node->getChildren())
        {
            auto result = findNodeWithJoint(child);
            if (result)
            {
                return result;
            }
        }
        return nullptr;
    };

    std::function<std::vector<pxr::UsdPrim>(const std::shared_ptr<RobotLinkNode>&)> collectForwardLinks =
        [&](const std::shared_ptr<RobotLinkNode>& node)
    {
        std::vector<pxr::UsdPrim> collected;
        if (!node || !node->getPrim())
        {
            return collected;
        }
        collected.push_back(node->getPrim());
        for (const auto& child : node->getChildren())
        {
            auto childLinks = collectForwardLinks(child);
            collected.insert(collected.end(), childLinks.begin(), childLinks.end());
        }
        return collected;
    };

    std::function<std::vector<pxr::UsdPrim>(const std::shared_ptr<RobotLinkNode>&)> collectBackwardLinks =
        [&](const std::shared_ptr<RobotLinkNode>& node)
    {
        std::vector<pxr::UsdPrim> collected;
        auto current = node;
        auto previous = node;
        while (current)
        {
            if (current->getPrim())
            {
                collected.push_back(current->getPrim());
            }
            auto parent = current->getParent();
            if (parent)
            {
                for (const auto& child : parent->getChildren())
                {
                    if (child == previous)
                    {
                        continue;
                    }
                    auto branchLinks = collectForwardLinks(child);
                    collected.insert(collected.end(), branchLinks.begin(), branchLinks.end());
                }
            }
            previous = current;
            current = parent;
        }
        return collected;
    };

    auto nodeAfterJoint = findNodeWithJoint(root);
    if (!nodeAfterJoint)
    {
        return { {}, {} };
    }

    const auto forwardLinks = collectForwardLinks(nodeAfterJoint);
    const auto backwardLinks = collectBackwardLinks(nodeAfterJoint->getParent());
    return { backwardLinks, forwardLinks };
}

/**
 * @brief Build a RobotLinkNode tree from a robot prim.
 * @details
 * Iteratively traverses joints referenced by each link to populate a tree
 * structure that mirrors the articulation hierarchy.
 *
 * @param[in] stage Stage that owns the prims.
 * @param[in] robotLinkPrim Root prim representing the robot.
 *
 * @return std::shared_ptr<RobotLinkNode> Tree root or nullptr on failure.
 */
inline std::shared_ptr<RobotLinkNode> GenerateRobotLinkTree(const pxr::UsdStagePtr& stage,
                                                            const pxr::UsdPrim& robotLinkPrim)
{
    if (!stage || !robotLinkPrim)
    {
        return nullptr;
    }

    std::vector<pxr::UsdPrim> allLinks;
    for (const pxr::UsdPrim& prim : pxr::UsdPrimRange(robotLinkPrim))
    {
        if (prim.HasAPI(className(Classes::LINK_API)))
        {
            allLinks.emplace_back(prim);
        }
    }

    std::vector<pxr::UsdPrim> allJoints;
    for (const pxr::UsdPrim& prim : pxr::UsdPrimRange(robotLinkPrim))
    {
        if (prim.HasAPI(className(Classes::JOINT_API)))
        {
            allJoints.emplace_back(prim);
        }
    }

    const auto links = GetAllRobotLinks(stage, robotLinkPrim);
    if (links.empty())
    {
        return nullptr;
    }

    auto root = std::make_shared<RobotLinkNode>(links.front());

    std::array<std::unordered_map<pxr::SdfPath, std::vector<pxr::UsdPrim>, pxr::SdfPath::Hash>, 2> jointsPerBody;
    for (const auto& link : allLinks)
    {
        const pxr::SdfPath path = link.GetPath();
        jointsPerBody[0].emplace(path, std::vector<pxr::UsdPrim>());
        jointsPerBody[1].emplace(path, std::vector<pxr::UsdPrim>());
    }

    for (const auto& joint : allJoints)
    {
        const pxr::SdfPath body0 = GetJointBodyRelationship(joint, 0);
        const pxr::SdfPath body1 = GetJointBodyRelationship(joint, 1);
        if (!body0.IsEmpty())
        {
            jointsPerBody[0][body0].push_back(joint);
        }
        if (!body1.IsEmpty())
        {
            jointsPerBody[1][body1].push_back(joint);
        }
    }

    std::vector<std::shared_ptr<RobotLinkNode>> stack = { root };
    std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash> processed;

    while (!stack.empty())
    {
        auto current = stack.back();
        stack.pop_back();
        const pxr::SdfPath currentPath = current->getPath();

        for (size_t index = 0; index < jointsPerBody.size(); ++index)
        {
            auto iterator = jointsPerBody[index].find(currentPath);
            if (iterator == jointsPerBody[index].end())
            {
                continue;
            }

            for (const auto& joint : iterator->second)
            {
                const pxr::SdfPath jointPath = joint.GetPath();
                if (!processed.insert(jointPath).second)
                {
                    continue;
                }

                const pxr::SdfPath childPath = GetJointBodyRelationship(joint, 1);
                if (childPath.IsEmpty())
                {
                    continue;
                }

                auto parent = current->getParent();
                if (parent && parent->getPath() == childPath)
                {
                    continue;
                }

                pxr::UsdPrim childPrim = stage->GetPrimAtPath(childPath);
                if (!childPrim)
                {
                    continue;
                }

                auto childNode = std::make_shared<RobotLinkNode>(childPrim, current, joint);
                current->addJoint(joint);
                current->addChild(childNode);
                stack.push_back(childNode);
            }
        }
    }

    return root;
}

}
}
}
