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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/robot/schema/robot_schema.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/robot/schema/utils.h>
#include <omni/fabric/FabricUSD.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec4d.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include <OgnIsaacComputeTransformTreeDatabase.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace nodes
{

using experimental::prims::IArticulationDataView;
using experimental::prims::IPrimDataReader;
using experimental::prims::IPrimDataReaderManager;
using experimental::prims::IXformDataView;

static std::atomic<int> s_transformTreeViewCounter{ 0 };

/// View index value meaning "use world or optional parent prim" (not an index into m_viewPaths).
static constexpr int kWorldParentViewIdx = -1;

/// Per-link-pair record: which view indices are the parent and child, plus cached tokens.
struct TransformPair
{
    int parentViewIdx; ///< kWorldParentViewIdx = world/external parent; >= 0 = index in m_viewPaths
    int childViewIdx; ///< index in m_viewPaths
    bool isCamera; ///< true if child is a UsdGeomCamera (needs 180° x-axis rotation for ROS convention)
    NameToken parentToken; ///< cached token (avoid per-frame stringToToken)
    NameToken childToken;
};

/// World-space position + orientation in float32, matching IPrimDataReader's float arrays directly.
struct WorldPose
{
    float px, py, pz;
    float qw, qx, qy, qz; ///< (w, x, y, z) to match Fabric decomposeMatrix layout
};

struct PrimInfo
{
    bool isPhysics = false;
    bool isCamera = false; ///< UsdGeomCamera (not an RTX Lidar sensor) — needs 180° x rotation
    int physicsViewIdx = -1;
    int physicsAncestorGlobalIdx = -1;
    std::vector<pxr::SdfPath> localChain;
};

/// Per-path local-transform read result. `resetsXformStack` is USD's `!resetXformStack!` flag;
/// when true the composition loop in `computeNonPhysicsWorldPose` must discard the accumulated
/// ancestor world and re-root from `matrix`, matching `ComputeLocalToWorldTransform` semantics.
struct LocalMatrixEntry
{
    pxr::GfMatrix4d matrix;
    bool resetsXformStack;
};

static constexpr WorldPose kIdentityPose{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };

/// Hamilton product: out = a * b.
static inline void quatMul(
    float aw, float ax, float ay, float az, float bw, float bx, float by, float bz, float& ow, float& ox, float& oy, float& oz)
{
    ow = aw * bw - ax * bx - ay * by - az * bz;
    ox = aw * bx + ax * bw + ay * bz - az * by;
    oy = aw * by - ax * bz + ay * bw + az * bx;
    oz = aw * bz + ax * by - ay * bx + az * bw;
}

/// Rotate vector (vx,vy,vz) by unit quaternion (qw,qx,qy,qz) using the
/// cross-product form: result = v + 2w*(q×v) + 2*(q×(q×v)).
static inline void rotateVec(
    float qw, float qx, float qy, float qz, float vx, float vy, float vz, float& ox, float& oy, float& oz)
{
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);
    ox = vx + qw * tx + (qy * tz - qz * ty);
    oy = vy + qw * ty + (qz * tx - qx * tz);
    oz = vz + qw * tz + (qx * ty - qy * tx);
}

/// Build a GfMatrix4d from a rigid translation+quaternion WorldPose.
static pxr::GfMatrix4d matrixFromWorldPose(const WorldPose& pose)
{
    pxr::GfMatrix4d m;
    m.SetIdentity();
    m.SetRotateOnly(pxr::GfQuatd(pose.qw, pose.qx, pose.qy, pose.qz));
    m.SetTranslateOnly(pxr::GfVec3d(pose.px, pose.py, pose.pz));
    return m;
}

/// Decompose a GfMatrix4d into a rigid translation+quaternion WorldPose. The rotation sub-matrix
/// is orthonormalized first so ancestor scale does not corrupt the output orientation. Scale is
/// intentionally dropped at the world-pose output boundary, but any ancestor scale already baked
/// into the translation is preserved.
static WorldPose worldPoseFromMatrix(const pxr::GfMatrix4d& m)
{
    pxr::GfVec3d t = m.ExtractTranslation();
    pxr::GfMatrix4d orthonormal = m;
    orthonormal.Orthonormalize(/*issueWarning=*/false);
    pxr::GfQuatd q = orthonormal.ExtractRotationQuat();
    pxr::GfVec3d imag = q.GetImaginary();
    return { static_cast<float>(t[0]),        static_cast<float>(t[1]),    static_cast<float>(t[2]),
             static_cast<float>(q.GetReal()), static_cast<float>(imag[0]), static_cast<float>(imag[1]),
             static_cast<float>(imag[2]) };
}

/// Read a WorldPose directly from the IXformDataView's float arrays with zero conversion.
static inline WorldPose worldPoseFromView(const float* positions, const float* orientations, int idx)
{
    return { positions[3 * idx],        positions[3 * idx + 1],    positions[3 * idx + 2],   orientations[4 * idx],
             orientations[4 * idx + 1], orientations[4 * idx + 2], orientations[4 * idx + 3] };
}

/// Compute child-in-parent relative transform using inline float32 quaternion math.
static inline void computeRelativeTransform(const WorldPose& parent,
                                            const WorldPose& child,
                                            float& outTx,
                                            float& outTy,
                                            float& outTz,
                                            float& outQw,
                                            float& outQx,
                                            float& outQy,
                                            float& outQz)
{
    float piw = parent.qw, pix = -parent.qx, piy = -parent.qy, piz = -parent.qz;
    quatMul(piw, pix, piy, piz, child.qw, child.qx, child.qy, child.qz, outQw, outQx, outQy, outQz);
    rotateVec(piw, pix, piy, piz, child.px - parent.px, child.py - parent.py, child.pz - parent.pz, outTx, outTy, outTz);
}

/// Apply the ROS optical-frame 180° x-axis rotation to a camera's world pose in place.
/// Converts from USD camera convention (looking down -Z) to ROS optical frame.
static inline void applyCameraRotation(WorldPose& pose)
{
    float cw, cx, cy, cz;
    quatMul(pose.qw, pose.qx, pose.qy, pose.qz, 0.0f, 1.0f, 0.0f, 0.0f, cw, cx, cy, cz);
    pose.qw = cw;
    pose.qx = cx;
    pose.qy = cy;
    pose.qz = cz;
}

class OgnIsaacComputeTransformTree : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacComputeTransformTree()
    {
        cleanupView();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacComputeTransformTreeDatabase::sPerInstanceState<OgnIsaacComputeTransformTree>(nodeObj, instanceId);
        state.m_simManager = carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();
        state.m_readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
    }

    static bool compute(OgnIsaacComputeTransformTreeDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] OgnIsaacComputeTransformTree::compute");
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnIsaacComputeTransformTree>();

        if (!state.ensureCurrentView(db, context))
        {
            return false;
        }

        if (state.m_viewPaths.empty())
        {
            return false;
        }

        return state.computeTransforms(db);
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnIsaacComputeTransformTreeDatabase::sPerInstanceState<OgnIsaacComputeTransformTree>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        cleanupView();
        m_firstFrame = true;
        m_viewPaths.clear();
        m_originalPrimCount = 0;
        m_pairs.clear();
        m_lastNumPairs = -1;
        m_parentPath.clear();
        m_parentFrame = "world";
        m_frameNamesStale = true;
        m_primInfo.clear();
        m_pathToGlobalIdx.clear();
        m_physicsViewPaths.clear();
        m_computedWorldPoses.clear();
        m_xformableCache.clear();
        m_localMatrixMemo.clear();
        m_hasNonPhysicsPrims = false;
        m_parentIsPhysics = false;
        m_parentPhysicsViewIdx = -1;
        m_parentPrimInfo = {};
        m_usdStage = nullptr;
    }

private:
    bool ensureCurrentView(OgnIsaacComputeTransformTreeDatabase& db, const GraphContextObj& context)
    {
        if (!m_simManager || !m_simManager->isSimulating())
        {
            return false;
        }

        if (m_firstFrame)
        {
            return initialize(db, context);
        }

        if (m_reader && m_reader->getGeneration() != m_readerGeneration)
        {
            reset();
            return initialize(db, context);
        }

        return !m_viewPaths.empty();
    }

    bool initialize(OgnIsaacComputeTransformTreeDatabase& db, const GraphContextObj& context)
    {
        CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::initialize");
        long stageId = context.iContext->getStageId(context);

        if (!createXformViewSetup(db, stageId))
        {
            return false;
        }

        std::unordered_map<std::string, std::string> linkParents;
        if (!collectViewPathsAndLinkParents(db, linkParents, stageId))
        {
            return false;
        }

        m_originalPrimCount = m_viewPaths.size();
        buildPhysicsViewAndAncestors();

        std::unordered_set<std::string> physicsPathSet(m_physicsViewPaths.begin(), m_physicsViewPaths.end());
        classifyParentPrim(db, physicsPathSet);

        if (!createXformView(db, stageId))
        {
            return false;
        }

        resolveParentFrameName();
        buildTransformPairs(linkParents, m_originalPrimCount);
        m_computedWorldPoses.resize(m_viewPaths.size(), kIdentityPose);
        populateXformableCache();
        m_firstFrame = false;
        m_frameNamesStale = true;
        return true;
    }

    /// Setup the reader/manager before resolving paths (called first in initialize()).
    bool createXformViewSetup(OgnIsaacComputeTransformTreeDatabase& db, long stageId)
    {
        if (!m_readerManager)
        {
            db.logError("Failed to acquire IPrimDataReaderManager interface");
            return false;
        }
        if (!m_readerManager->ensureInitialized(stageId, -1))
        {
            db.logError("Failed to initialize shared prim data reader session");
            return false;
        }
        m_reader = m_readerManager->getReader();
        if (!m_reader)
        {
            db.logError("Failed to acquire shared IPrimDataReader interface");
            return false;
        }
        m_readerGeneration = m_reader->getGeneration();
        return true;
    }

    /// Resolve a frame name for @p primPath from the USD stage: honor `isaac:nameOverride`, fall
    /// back to the USD leaf name. Works on both the tensor-backed and pure-USD paths because it
    /// does not require an IXformDataView (the view only exists when the node has physics prims).
    std::string resolveFrameName(const std::string& primPath)
    {
        if (m_usdStage)
        {
            if (pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(primPath)))
            {
                return isaacsim::core::includes::getName(prim);
            }
        }
        auto slash = primPath.rfind('/');
        return (slash != std::string::npos) ? primPath.substr(slash + 1) : primPath;
    }

    void resolveParentFrameName()
    {
        if (m_parentPath.empty())
        {
            return;
        }
        m_parentFrame = resolveFrameName(m_parentPath);
    }

    bool collectViewPathsAndLinkParents(OgnIsaacComputeTransformTreeDatabase& db,
                                        std::unordered_map<std::string, std::string>& linkParents,
                                        long stageId)
    {
        const auto& targetPrims = db.inputs.targetPrims();
        if (targetPrims.empty())
        {
            db.logError("Please specify at least one valid target prim");
            return false;
        }

        // Obtain the stage to guard articulation discovery with a cheap API check, avoiding
        // spurious PhysX tensor errors for sensor/camera/Xform-only prims. Store only a weak
        // pointer on the node so it does not extend the stage lifetime after initialization.
        m_usdStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        if (!m_usdStage)
        {
            db.logError("Could not find USD stage %ld", stageId);
            return false;
        }

        for (size_t i = 0; i < targetPrims.size(); i++)
        {
            std::string primPathStr = omni::fabric::toSdfPath(targetPrims[i]).GetString();

            // Guard against empty or non-absolute paths from disconnected/deleted target prim definitions.
            if (primPathStr.empty() || primPathStr[0] != '/')
            {
                CARB_LOG_WARN("IsaacComputeTransformTree: skipping target prim at index %zu with invalid path '%s'", i,
                              primPathStr.c_str());
                continue;
            }

            // Look up prim once for both articulation check and camera detection below.
            pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(primPathStr));
            if (!prim)
            {
                CARB_LOG_WARN("IsaacComputeTransformTree: prim '%s' not found on stage, skipping", primPathStr.c_str());
                continue;
            }

            // Only attempt articulation discovery for prims that have UsdPhysicsArticulationRootAPI.
            // Without this guard, createArticulationView triggers PhysX tensor errors for every
            // non-physics prim (sensors, cameras, IMUs, etc.).
            bool hasArticulationApi = prim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>();

            bool isArticulation = false;
            if (hasArticulationApi)
            {
                // Create a temporary articulation view to detect links via USD traversal.
                // Use "newton" engine type to avoid triggering PhysX tensor setup — getArticulationLinks
                // is pure USD traversal and does not require any physics backend to be active.
                std::string tempId = "ctt_discover_" + std::to_string(i);
                const char* pathPtr = primPathStr.c_str();
                IArticulationDataView* artView = m_reader->createArticulationView(tempId.c_str(), &pathPtr, 1, "newton");

                const experimental::prims::LinkInfo* links = nullptr;
                size_t linkCount = 0;
                isArticulation =
                    artView && artView->getArticulationLinks(primPathStr.c_str(), &links, &linkCount) && linkCount > 0;

                if (isArticulation)
                {
                    // Copy link data before removing the view (pointers become invalid after removeView).
                    std::vector<std::pair<std::string, std::string>> copied(linkCount);
                    for (size_t j = 0; j < linkCount; j++)
                    {
                        copied[j] = { links[j].path, links[j].parentPath };
                    }
                    m_reader->removeView(tempId.c_str());
                    for (const auto& [path, parentPath] : copied)
                    {
                        m_viewPaths.push_back(path);
                        PrimInfo info;
                        info.isPhysics = true;
                        m_primInfo.push_back(info);
                        linkParents[path] = parentPath;
                    }
                }
                else
                {
                    m_reader->removeView(tempId.c_str());
                }
            }

            // Fallback: target prim carries IsaacRobotAPI but no UsdPhysicsArticulationRootAPI;
            // enumerate every Isaac-schema-tagged prim under the robot via the central
            // `GetAllRobotComponents` helper (links + sites + reference points + nested robots).
            if (!isArticulation &&
                prim.HasAPI(isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::ROBOT_API)))
            {
                // Reconstruct the kinematic child→parent map from `isaac:physics:robotJoints` so
                // emitted TF pairs mirror the joint graph. Root links keep an empty parent and
                // re-parent to world (or the user-supplied parentPrim) downstream.
                const auto jointParentMap = isaacsim::robot::schema::GetRobotLinkParentMap(m_usdStage, prim);

                // Resolve the user-supplied parentPrim once so we can skip emitting a
                // `parentPrim → parentPrim` self-pair that ROS rejects with `TF_SELF_TRANSFORM`.
                std::string parentPrimPathStr;
                const auto& parentPrimInput = db.inputs.parentPrim();
                if (!parentPrimInput.empty())
                {
                    parentPrimPathStr = omni::fabric::toSdfPath(parentPrimInput[0]).GetString();
                }

                const pxr::TfToken linkApiToken =
                    isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::LINK_API);
                const pxr::TfToken jointApiToken =
                    isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::JOINT_API);
                const pxr::TfToken robotApiToken =
                    isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::ROBOT_API);

                for (const auto& component : isaacsim::robot::schema::GetAllRobotComponents(m_usdStage, prim))
                {
                    // Joints are constraints, not frames — skip them.
                    if (component.HasAPI(jointApiToken))
                    {
                        continue;
                    }
                    // Nested IsaacRobotAPI roots are aggregations, not frames themselves; their
                    // child links/sites are already in the list separately.
                    if (component.HasAPI(robotApiToken))
                    {
                        continue;
                    }
                    const std::string componentPathStr = component.GetPath().GetString();
                    // Skip the link that coincides with `parentPrim` to avoid a self-loop pair.
                    if (!parentPrimPathStr.empty() && componentPathStr == parentPrimPathStr)
                    {
                        isArticulation = true;
                        continue;
                    }

                    m_viewPaths.push_back(componentPathStr);
                    PrimInfo info;
                    info.isPhysics = component.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();
                    if (component.IsA<pxr::UsdGeomCamera>())
                    {
                        using namespace isaacsim::robot::schema::sensors;
                        info.isCamera = !component.HasAPI(kIsaacRtxLidarSensorAPI);
                    }
                    if (!info.isPhysics)
                    {
                        m_hasNonPhysicsPrims = true;
                    }
                    m_primInfo.push_back(info);

                    // Parent resolution:
                    //  - IsaacLinkAPI prims: use the joint topology (body0 → body1 map).
                    //  - Other (sites, reference points): use the nearest ancestor link.
                    std::string parentPath;
                    if (component.HasAPI(linkApiToken))
                    {
                        auto pit = jointParentMap.find(componentPathStr);
                        if (pit != jointParentMap.end())
                        {
                            parentPath = pit->second;
                        }
                    }
                    else
                    {
                        pxr::UsdPrim ancestor = component.GetParent();
                        while (ancestor)
                        {
                            if (ancestor.HasAPI(linkApiToken))
                            {
                                parentPath = ancestor.GetPath().GetString();
                                break;
                            }
                            ancestor = ancestor.GetParent();
                        }
                    }
                    linkParents[componentPathStr] = parentPath;
                    isArticulation = true;
                }
            }

            if (!isArticulation)
            {
                // Not an articulation (or no links found): treat as a single prim.
                m_viewPaths.push_back(primPathStr);
                linkParents[primPathStr] = "";

                PrimInfo info;
                info.isPhysics = prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();

                // Cameras (excluding RTX Lidar sensors that share the UsdGeomCamera schema)
                // need a 180° x-axis rotation to match the ROS optical frame convention.
                if (prim.IsA<pxr::UsdGeomCamera>())
                {
                    using namespace isaacsim::robot::schema::sensors;
                    info.isCamera = !prim.HasAPI(kIsaacRtxLidarSensorAPI);
                }
                if (!info.isPhysics)
                {
                    m_hasNonPhysicsPrims = true;
                }
                m_primInfo.push_back(info);
            }
        }

        if (m_viewPaths.empty())
        {
            db.logError("No valid prims found after resolving target prims");
            return false;
        }
        return true;
    }

    void buildPhysicsViewAndAncestors()
    {
        std::unordered_set<std::string> physicsPathSet;
        for (size_t i = 0; i < m_viewPaths.size(); i++)
        {
            if (m_primInfo[i].isPhysics)
            {
                m_primInfo[i].physicsViewIdx = static_cast<int>(m_physicsViewPaths.size());
                m_physicsViewPaths.push_back(m_viewPaths[i]);
                physicsPathSet.insert(m_viewPaths[i]);
            }
        }

        m_pathToGlobalIdx.clear();
        for (size_t i = 0; i < m_viewPaths.size(); i++)
        {
            m_pathToGlobalIdx[m_viewPaths[i]] = static_cast<int>(i);
        }

        if (!m_hasNonPhysicsPrims)
        {
            return;
        }

        // Index-based iteration is intentional: discoverPhysicsAncestor may append new physics
        // ancestors to m_viewPaths/m_primInfo, but those are always isPhysics==true and skipped.
        for (size_t i = 0; i < m_viewPaths.size(); i++)
        {
            if (m_primInfo[i].isPhysics)
            {
                continue;
            }
            discoverPhysicsAncestor(i, physicsPathSet);
        }
    }

    /// Append @p path as a new physics prim to the view bookkeeping (m_viewPaths / m_physicsViewPaths /
    /// m_primInfo / m_pathToGlobalIdx) and record it in @p physicsPathSet. Returns the new global
    /// index. Used by both target-chain and parent-chain ancestor auto-discovery.
    int promotePhysicsAncestor(const pxr::SdfPath& path, std::unordered_set<std::string>& physicsPathSet)
    {
        const std::string pathStr = path.GetString();
        int newGlobalIdx = static_cast<int>(m_viewPaths.size());
        m_viewPaths.push_back(pathStr);
        PrimInfo info;
        info.isPhysics = true;
        info.physicsViewIdx = static_cast<int>(m_physicsViewPaths.size());
        m_primInfo.push_back(info);
        m_physicsViewPaths.push_back(pathStr);
        physicsPathSet.insert(pathStr);
        m_pathToGlobalIdx[pathStr] = newGlobalIdx;
        return newGlobalIdx;
    }

    /// Walk ancestors of @p startPath toward the stage root looking for the first physics rigid
    /// body. The walked chain (ordered root→leaf) is returned via @p outChain. Returns the global
    /// index of the physics ancestor, or -1 if none found. A physics ancestor already tracked in
    /// @p physicsPathSet is reused; an untracked ancestor with `UsdPhysicsRigidBodyAPI` is
    /// auto-promoted into the view via `promotePhysicsAncestor`.
    int walkForPhysicsAncestor(const pxr::SdfPath& startPath,
                               std::unordered_set<std::string>& physicsPathSet,
                               std::vector<pxr::SdfPath>& outChain)
    {
        outChain.clear();
        pxr::SdfPath current = startPath;
        while (true)
        {
            outChain.push_back(current);
            pxr::SdfPath parentPath = current.GetParentPath();
            if (parentPath.IsEmpty() || parentPath == pxr::SdfPath::AbsoluteRootPath())
            {
                break;
            }
            const std::string parentStr = parentPath.GetString();
            if (physicsPathSet.count(parentStr))
            {
                std::reverse(outChain.begin(), outChain.end());
                auto it = m_pathToGlobalIdx.find(parentStr);
                return (it != m_pathToGlobalIdx.end()) ? it->second : -1;
            }
            if (m_usdStage)
            {
                pxr::UsdPrim ancestorPrim = m_usdStage->GetPrimAtPath(parentPath);
                if (ancestorPrim && ancestorPrim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
                {
                    int newGlobalIdx = promotePhysicsAncestor(parentPath, physicsPathSet);
                    std::reverse(outChain.begin(), outChain.end());
                    return newGlobalIdx;
                }
            }
            current = parentPath;
        }
        std::reverse(outChain.begin(), outChain.end());
        return -1;
    }

    void discoverPhysicsAncestor(size_t globalIdx, std::unordered_set<std::string>& physicsPathSet)
    {
        // Walk into a local chain first: `walkForPhysicsAncestor` may call `promotePhysicsAncestor`
        // which `push_back`s into `m_primInfo` and can invalidate any reference into the vector.
        // We therefore avoid holding `m_primInfo[globalIdx]` across the walk.
        std::vector<pxr::SdfPath> chain;
        int ancestorIdx = walkForPhysicsAncestor(pxr::SdfPath(m_viewPaths[globalIdx]), physicsPathSet, chain);
        m_primInfo[globalIdx].physicsAncestorGlobalIdx = ancestorIdx;
        m_primInfo[globalIdx].localChain = std::move(chain);
    }

    bool createXformView(OgnIsaacComputeTransformTreeDatabase& db, long /*stageId*/)
    {
        if (m_physicsViewPaths.empty())
        {
            return true;
        }
        m_viewId = "compute_transform_tree_" + std::to_string(s_transformTreeViewCounter.fetch_add(1));
        std::vector<const char*> pathPtrs;
        pathPtrs.reserve(m_physicsViewPaths.size());
        for (const auto& p : m_physicsViewPaths)
        {
            pathPtrs.push_back(p.c_str());
        }
        const char* engine = isaacsim::core::includes::getActivePhysicsEngineName();
        m_xformView = m_reader->createXformView(m_viewId.c_str(), pathPtrs.data(), pathPtrs.size(), engine);
        if (!m_xformView)
        {
            db.logError("Failed to create xform view");
            return false;
        }
        return true;
    }

    /// Read the authored local transform of @p path along with the USD `resetXformStack` flag.
    /// The flag is set when the prim has a `!resetXformStack!` op in its xformop order: callers
    /// must treat the returned local matrix as the WORLD matrix (i.e. ignore the ancestor chain)
    /// to match USD's `ComputeLocalToWorldTransform` semantics.
    ///
    /// Two-layer cache: @ref m_localMatrixMemo (per-frame memo, cleared at start of each
    /// `composeNonPhysicsPoses` pass) short-circuits repeated queries for shared ancestors across
    /// sibling chains. On a memo miss, the cached `UsdGeomXformable` in @ref m_xformableCache
    /// (populated once by `populateXformableCache` at the end of `initialize()`) lets us skip
    /// `stage->GetPrimAtPath()` and adapter construction, doing only the `GetLocalTransformation`
    /// USD query. Callers must ensure `path` is present in @ref m_xformableCache (which holds
    /// every path that can appear in any local chain).
    LocalMatrixEntry readLocalMatrix(const pxr::SdfPath& path)
    {
        if (auto memoIt = m_localMatrixMemo.find(path); memoIt != m_localMatrixMemo.end())
        {
            return memoIt->second;
        }

        LocalMatrixEntry entry;
        entry.matrix.SetIdentity();
        entry.resetsXformStack = false;

        if (auto cacheIt = m_xformableCache.find(path); cacheIt != m_xformableCache.end() && cacheIt->second)
        {
            // UsdTimeCode::Default() reads the authored (rest-pose) local transform — correct for
            // static sensor offsets. Animated local transforms would require a timeSampled path.
            if (!cacheIt->second.GetLocalTransformation(
                    &entry.matrix, &entry.resetsXformStack, pxr::UsdTimeCode::Default()))
            {
                entry.matrix.SetIdentity();
                entry.resetsXformStack = false;
            }
        }

        m_localMatrixMemo.emplace(path, entry);
        return entry;
    }

    /// Populate @ref m_xformableCache with a `UsdGeomXformable` for every unique SdfPath that
    /// appears in any non-physics local chain (including the parent chain). Called once at the
    /// end of initialize() so the per-frame hot path never has to do a `stage->GetPrimAtPath()`
    /// or construct a `UsdGeomXformable` adapter.
    void populateXformableCache()
    {
        m_xformableCache.clear();
        if (!m_usdStage)
        {
            return;
        }
        auto addChain = [this](const std::vector<pxr::SdfPath>& chain)
        {
            for (const auto& path : chain)
            {
                if (m_xformableCache.find(path) != m_xformableCache.end())
                {
                    continue;
                }
                pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(path);
                m_xformableCache.emplace(path, prim ? pxr::UsdGeomXformable(prim) : pxr::UsdGeomXformable());
            }
        };
        for (const auto& info : m_primInfo)
        {
            if (!info.isPhysics && !info.localChain.empty())
            {
                addChain(info.localChain);
            }
        }
        if (!m_parentPrimInfo.localChain.empty())
        {
            addChain(m_parentPrimInfo.localChain);
        }
    }

    /// Compose the physics-ancestor world pose with each authored local matrix in the chain using
    /// full 4x4 matrix multiplication, then decompose once at the end. This preserves scale and
    /// shear semantics from non-physics ancestors (e.g. a scaled mount correctly scales a child
    /// sensor's translation), matching the pre-optimization world-matrix path.
    ///
    /// If any prim in the chain has `!resetXformStack!` authored, the accumulated ancestor world
    /// is discarded from that prim downward — same semantics as `UsdGeomXformable::
    /// ComputeLocalToWorldTransform`.
    WorldPose computeNonPhysicsWorldPose(const PrimInfo& info)
    {
        pxr::GfMatrix4d world = (info.physicsAncestorGlobalIdx >= 0) ?
                                    matrixFromWorldPose(m_computedWorldPoses[info.physicsAncestorGlobalIdx]) :
                                    pxr::GfMatrix4d(1.0);

        for (const auto& path : info.localChain)
        {
            const LocalMatrixEntry& entry = readLocalMatrix(path);
            // Row-vector convention: v_world = v_local * M_local * M_parent_world, so child-to-world
            // is local * world. When the prim resets the xform stack, USD defines the prim's world
            // to be just its local matrix (no ancestor contribution), so we overwrite `world`.
            world = entry.resetsXformStack ? entry.matrix : (entry.matrix * world);
        }
        return worldPoseFromMatrix(world);
    }

    void classifyParentPrim(OgnIsaacComputeTransformTreeDatabase& db, std::unordered_set<std::string>& physicsPathSet)
    {
        const auto& parentPrimInput = db.inputs.parentPrim();
        if (parentPrimInput.empty())
        {
            return;
        }

        m_parentPath = omni::fabric::toSdfPath(parentPrimInput[0]).GetString();

        // Case 1: parent is a target already in the view AND classified as physics → reuse idx.
        if (auto it = m_pathToGlobalIdx.find(m_parentPath);
            it != m_pathToGlobalIdx.end() && m_primInfo[it->second].isPhysics)
        {
            m_parentIsPhysics = true;
            m_parentPhysicsViewIdx = m_primInfo[it->second].physicsViewIdx;
            return;
        }

        // Case 2: parent is itself a physics rigid body not yet tracked as a target → promote it
        // into the physics view so we can read its tensor pose. The parent is only needed for
        // pose (not emitted as an output pair), so we don't add it to m_viewPaths. Case 1 already
        // returned if the parent is both a target and classified as physics, and `physicsPathSet`
        // is always populated in lock-step with `m_primInfo[i].isPhysics`, so the insert must
        // succeed here.
        if (m_usdStage)
        {
            pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(m_parentPath));
            if (prim && prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
            {
                m_parentIsPhysics = true;
                physicsPathSet.insert(m_parentPath);
                m_parentPhysicsViewIdx = static_cast<int>(m_physicsViewPaths.size());
                m_physicsViewPaths.push_back(m_parentPath);
                return;
            }
        }

        // Case 3: parent is non-physics. Walk ancestors; if one of them is a physics rigid body,
        // track it so the parent's world pose follows runtime physics motion (this requires
        // auto-promoting the ancestor when it's not already in the view). Otherwise fall back to
        // composing authored local transforms only.
        m_parentPrimInfo = {};
        m_parentPrimInfo.physicsAncestorGlobalIdx =
            walkForPhysicsAncestor(pxr::SdfPath(m_parentPath), physicsPathSet, m_parentPrimInfo.localChain);
    }

    // Records only pair topology (indices + isCamera). Frame names and tokens are filled by rebuildFrameNames().
    // @p originalPrimCount limits iteration to user-provided target prims (excluding auto-discovered physics
    // ancestors).
    void buildTransformPairs(const std::unordered_map<std::string, std::string>& linkParents, size_t originalPrimCount)
    {
        for (size_t i = 0; i < originalPrimCount; i++)
        {
            const std::string& childPath = m_viewPaths[i];
            const std::string& parentPath = linkParents.at(childPath);
            const bool isCamera = m_primInfo[i].isCamera;

            int parentViewIdx = kWorldParentViewIdx;
            if (!parentPath.empty())
            {
                auto it = m_pathToGlobalIdx.find(parentPath);
                if (it != m_pathToGlobalIdx.end())
                {
                    parentViewIdx = it->second;
                }
            }
            m_pairs.push_back({ parentViewIdx, static_cast<int>(i), isCamera });
        }
    }

    /// Re-resolves frame names from the USD stage (after `isaac:nameOverride` authoring has
    /// settled) and updates m_pairs tokens. Called on the first compute frame after initialize().
    /// Each node instance resolves names independently, so multiple robots can each own frame
    /// names like 'front_fisheye_camera' without cross-node interference.
    ///
    /// Naming is computed only over the user-provided targets `[0, m_originalPrimCount)`:
    /// auto-discovered physics ancestors are never emitted as child pairs, so they don't
    /// participate in deepest-path preference or leaf-name disambiguation. When an ancestor is
    /// referenced as a pair's parent, it gets a simple USD-derived name with no warnings.
    void rebuildFrameNames(OgnIsaacComputeTransformTreeDatabase& db)
    {
        if (!m_parentPath.empty())
        {
            m_parentFrame = resolveFrameName(m_parentPath);
        }

        const size_t n = m_viewPaths.size();
        const size_t numTargets = m_originalPrimCount;
        std::vector<std::string> frameNames(n);

        // --- Targets only: apply nameOverride + deepest-path preference + leaf disambiguation. ---
        std::vector<std::string> desired(numTargets);
        for (size_t i = 0; i < numTargets; ++i)
        {
            desired[i] = resolveFrameName(m_viewPaths[i]);
        }

        // Deepest-path preference: for each desired name, record the target with the longest path
        // (sensor leaves take priority over mount parents with the same nameOverride).
        std::unordered_map<std::string, size_t> deepest;
        for (size_t i = 0; i < numTargets; ++i)
        {
            auto [it, inserted] = deepest.emplace(desired[i], i);
            if (!inserted && m_viewPaths[i].size() > m_viewPaths[it->second].size())
            {
                it->second = i;
            }
        }

        for (size_t i = 0; i < numTargets; ++i)
        {
            if (deepest.at(desired[i]) == i)
            {
                frameNames[i] = desired[i];
            }
        }

        // Non-primaries (e.g. mount prims that lost leaf-preference) fall back to USD leaf name.
        // If that is also taken (two prims share a USD leaf name), walk ancestors for a qualified name.
        const size_t startPos = firstComponentEnd();
        std::unordered_map<std::string, std::string> nameCache;
        for (size_t i = 0; i < numTargets; ++i)
        {
            if (!frameNames[i].empty())
            {
                continue;
            }

            const std::string& path = m_viewPaths[i];
            auto slash = path.rfind('/');
            std::string leafName = (slash != std::string::npos) ? path.substr(slash + 1) : path;

            std::string name = std::find(frameNames.begin(), frameNames.end(), leafName) != frameNames.end() ?
                                   disambiguateFrameName(path, leafName, startPos, frameNames, nameCache) :
                                   leafName;

            if (desired[i] != name)
            {
                CARB_LOG_WARN(
                    "Frame '%s' already exists (used by another prim). Using '%s' for '%s'. "
                    "Set unique `isaac:nameOverride` values per robot instance to suppress this.",
                    desired[i].c_str(), name.c_str(), path.c_str());
            }

            frameNames[i] = name;
        }

        // --- Promoted physics ancestors: simple USD-derived name for parent tokens. --------------
        // They never appear as a pair's child, so they don't need deepest-path or disambiguation
        // logic — and we must not log collision warnings for paths the user didn't request.
        for (size_t i = numTargets; i < n; ++i)
        {
            frameNames[i] = resolveFrameName(m_viewPaths[i]);
        }

        for (auto& pair : m_pairs)
        {
            pair.childToken = db.stringToToken(frameNames[pair.childViewIdx].c_str());
            const std::string& parentName =
                (pair.parentViewIdx != kWorldParentViewIdx) ? frameNames[pair.parentViewIdx] : m_parentFrame;
            pair.parentToken = db.stringToToken(parentName.c_str());
        }

        m_frameNamesStale = false;
    }

    bool computeTransforms(OgnIsaacComputeTransformTreeDatabase& db)
    {
        CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::computeTransforms");
        assert(m_primInfo.size() == m_viewPaths.size());
        assert(m_computedWorldPoses.size() == m_viewPaths.size());

        if (m_frameNamesStale)
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::rebuildFrameNames");
            rebuildFrameNames(db);
        }

        const float* positions = nullptr;
        const float* orientations = nullptr;
        if (m_xformView)
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::readPhysicsPoses");
            const auto poses = m_xformView->getWorldPosesHost();
            positions = poses.positions;
            orientations = poses.orientations;
            const size_t numPhysics = m_physicsViewPaths.size();
            if (!positions || !orientations || static_cast<size_t>(poses.posCount) < numPhysics * 3 ||
                static_cast<size_t>(poses.oriCount) < numPhysics * 4)
            {
                return false;
            }

            for (size_t i = 0; i < m_viewPaths.size(); i++)
            {
                const PrimInfo& info = m_primInfo[i];
                if (info.isPhysics)
                {
                    m_computedWorldPoses[i] = worldPoseFromView(positions, orientations, info.physicsViewIdx);
                }
            }
        }

        const bool hasNonPhysicsParent = !m_parentPath.empty() && !m_parentIsPhysics;
        if (m_hasNonPhysicsPrims || hasNonPhysicsParent)
        {
            // Clear per-frame memo before any USD local-chain composition. This includes the
            // parent-only path where all targets are physics but parentPrim is non-physics.
            m_localMatrixMemo.clear();
        }

        if (m_hasNonPhysicsPrims)
        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::composeNonPhysicsPoses");
            // Memo entries are re-populated lazily by readLocalMatrix. Shared ancestors across
            // sibling chains (e.g. a common sensor mount) are read from USD once per frame instead
            // of once per non-physics target.
            for (size_t i = 0; i < m_viewPaths.size(); i++)
            {
                const PrimInfo& info = m_primInfo[i];
                if (info.isPhysics)
                {
                    continue;
                }
                // `walkForPhysicsAncestor` always seeds `localChain` with the prim's own path, so
                // every non-physics prim is guaranteed a chain containing at least itself.
                assert(!info.localChain.empty());
                m_computedWorldPoses[i] = computeNonPhysicsWorldPose(info);
            }
        }

        WorldPose parentWorldPose = kIdentityPose;
        if (!m_parentPath.empty())
        {
            if (m_parentIsPhysics)
            {
                // classifyParentPrim guarantees a valid physics view idx when m_parentIsPhysics,
                // and `readPhysicsPoses` above would have returned false if positions/orientations
                // were missing with any physics prims in the view.
                assert(m_parentPhysicsViewIdx >= 0 && positions && orientations);
                parentWorldPose = worldPoseFromView(positions, orientations, m_parentPhysicsViewIdx);
            }
            else
            {
                // Case 3 of classifyParentPrim seeds the parent's localChain with m_parentPath
                // itself even when no physics ancestor exists.
                assert(!m_parentPrimInfo.localChain.empty());
                parentWorldPose = computeNonPhysicsWorldPose(m_parentPrimInfo);
            }
        }

        const int numPairs = static_cast<int>(m_pairs.size());

        auto& outParentFrames = db.outputs.parentFrames();
        auto& outChildFrames = db.outputs.childFrames();
        auto& outTranslations = db.outputs.translations();
        auto& outOrientations = db.outputs.orientations();

        if (numPairs != m_lastNumPairs)
        {
            outParentFrames.resize(numPairs);
            outChildFrames.resize(numPairs);
            outTranslations.resize(numPairs);
            outOrientations.resize(numPairs);
            m_lastNumPairs = numPairs;
        }

        {
            CARB_PROFILE_ZONE(1, "[IsaacSim] ComputeTransformTree::writePairs");
            for (int i = 0; i < numPairs; i++)
            {
                const TransformPair& pair = m_pairs[i];

                // Reference the cached child/parent world poses directly; only make a local copy
                // of the child when it's a camera (for the 180° x-axis rotation).
                const WorldPose& childRef = m_computedWorldPoses[pair.childViewIdx];
                WorldPose childLocal;
                const WorldPose* childPose = &childRef;
                if (pair.isCamera)
                {
                    childLocal = childRef;
                    applyCameraRotation(childLocal);
                    childPose = &childLocal;
                }

                const WorldPose& parentPose = (pair.parentViewIdx == kWorldParentViewIdx) ?
                                                  parentWorldPose :
                                                  m_computedWorldPoses[pair.parentViewIdx];

                float relTx, relTy, relTz, relQw, relQx, relQy, relQz;
                computeRelativeTransform(parentPose, *childPose, relTx, relTy, relTz, relQw, relQx, relQy, relQz);

                outParentFrames[i] = pair.parentToken;
                outChildFrames[i] = pair.childToken;
                outTranslations[i] = pxr::GfVec3d(relTx, relTy, relTz);
                outOrientations[i] = pxr::GfVec4d(relQx, relQy, relQz, relQw);
            }
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    /// Returns the byte offset of the slash that ends the first path component (e.g. 6 for
    /// "/World/..."). Used by disambiguateFrameName so the first ancestor qualifier tried is the
    /// robot-level prim (e.g. /World/Nova_Carter_ROS_1), which is unique per robot instance.
    size_t firstComponentEnd() const
    {
        if (m_viewPaths.empty())
        {
            return 0;
        }
        size_t pos = m_viewPaths[0].find('/', 1);
        return (pos != std::string::npos) ? pos : 0;
    }

    /// Builds a qualified frame name by prepending ancestor path components to @p leaf, starting
    /// past the first path component (e.g. past /World). Returns the first unused candidate,
    /// or a path slug from @p startPos as a last resort.
    std::string disambiguateFrameName(const std::string& path,
                                      const std::string& leaf,
                                      size_t startPos,
                                      const std::vector<std::string>& frameNames,
                                      std::unordered_map<std::string, std::string>& nameCache)
    {
        auto taken = [&](const std::string& s)
        { return std::find(frameNames.begin(), frameNames.end(), s) != frameNames.end(); };
        if (path.size() > 1 && path[0] == '/')
        {
            size_t pos = startPos;
            while (true)
            {
                size_t nextSlash = path.find('/', pos + 1);
                if (nextSlash == std::string::npos)
                {
                    break;
                }
                std::string ancestorPath = path.substr(0, nextSlash);
                auto it = nameCache.find(ancestorPath);
                std::string ancestorName;
                if (it != nameCache.end())
                {
                    ancestorName = it->second;
                }
                else
                {
                    ancestorName = resolveFrameName(ancestorPath);
                    if (ancestorName.empty())
                    {
                        ancestorName = path.substr(pos + 1, nextSlash - pos - 1);
                    }
                    nameCache[ancestorPath] = ancestorName;
                }
                if (!ancestorName.empty() && ancestorName != leaf)
                {
                    std::string candidate = ancestorName + "_" + leaf;
                    if (!taken(candidate))
                    {
                        return candidate;
                    }
                }
                pos = nextSlash;
            }
        }
        // Last resort: slug the path from startPos onward, skipping the root component.
        size_t slugStart = (startPos < path.size() && path[startPos] == '/') ? startPos + 1 : startPos;
        std::string name = path.substr(slugStart);
        std::replace(name.begin(), name.end(), '/', '_');
        return name;
    }

    void cleanupView()
    {
        if (m_reader && !m_viewId.empty())
        {
            m_reader->removeView(m_viewId.c_str());
        }
        m_xformView = nullptr;
        m_reader = nullptr;
        m_readerGeneration = 0;
        m_viewId.clear();
    }

    // Interfaces
    IPrimDataReaderManager* m_readerManager = nullptr;
    IPrimDataReader* m_reader = nullptr;
    IXformDataView* m_xformView = nullptr;
    isaacsim::core::simulation_manager::ISimulationManager* m_simManager = nullptr;
    uint64_t m_readerGeneration = 0;

    // View state
    std::string m_viewId;
    std::vector<std::string> m_viewPaths; ///< All prim paths in the view (targets + auto-promoted physics ancestors)
    size_t m_originalPrimCount = 0; ///< Count of user-provided target prims; prefix of @ref m_viewPaths
    std::vector<TransformPair> m_pairs; ///< Output parent-child pairs
    // Physics-aware optimization state
    std::vector<PrimInfo> m_primInfo;
    std::unordered_map<std::string, int> m_pathToGlobalIdx;
    std::vector<std::string> m_physicsViewPaths;
    std::vector<WorldPose> m_computedWorldPoses;
    /// `UsdGeomXformable` handles for every SdfPath that appears in any non-physics local chain.
    /// Populated once at initialize() so the per-frame hot path (`readLocalMatrix`) avoids
    /// `stage->GetPrimAtPath()` and `UsdGeomXformable` adapter construction on every chain element
    /// on every frame. Shared ancestors across multiple chains (common for sensor-mount hierarchies)
    /// get a single cache entry.
    std::unordered_map<pxr::SdfPath, pxr::UsdGeomXformable, pxr::SdfPath::Hash> m_xformableCache;
    /// Per-frame memoization of authored local matrices + `resetXformStack` flag, reused across
    /// sibling chains that share ancestors. Populated during `composeNonPhysicsPoses`; cleared
    /// at the start of every `computeTransforms` call so the next frame always reflects current
    /// USD authoring.
    std::unordered_map<pxr::SdfPath, LocalMatrixEntry, pxr::SdfPath::Hash> m_localMatrixMemo;
    bool m_hasNonPhysicsPrims = false;
    bool m_parentIsPhysics = false;
    int m_parentPhysicsViewIdx = -1;
    PrimInfo m_parentPrimInfo;
    /// Weak reference to the USD stage. Weak (not strong) so the node does not extend the
    /// lifetime of the stage beyond close/reload — the stage cache owns the stage, and a stage
    /// close should fully release it even if a node instance is still resident. Every access
    /// must null-check via `if (m_usdStage)`; a closed stage silently becomes null.
    pxr::UsdStageWeakPtr m_usdStage;

    // Parent frame
    std::string m_parentPath;
    std::string m_parentFrame = "world";

    bool m_firstFrame = true;
    bool m_frameNamesStale = true;
    int m_lastNumPairs = -1;
};

REGISTER_OGN_NODE()
} // namespace nodes
} // namespace core
} // namespace isaacsim
